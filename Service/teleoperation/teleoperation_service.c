#include "teleoperation_service.h"
#include "platform_critical.h"
#include "platform_time.h"

#include "command_management_service.h"

#include "ps2_controller_driver.h"

#include "relative_heading_controller.h"

#include "state_estimation_service.h"

#define PS2_DPAD_UP_MASK    0x10U
#define PS2_DPAD_RIGHT_MASK 0x20U
#define PS2_DPAD_DOWN_MASK  0x40U
#define PS2_DPAD_LEFT_MASK  0x80U

typedef teleoperation_status_t ps2_control_service_state_t;

static teleoperation_config_t        teleoperation_config;
static ps2_control_service_state_t   ps2_state;
static uint8_t                       last_btn2;
static uint8_t                       consecutive_read_failures;
static relative_heading_controller_t heading_control;
static uint8_t                       heading_button;
static uint8_t                       heading_zero_pending;
static uint32_t                      heading_motion_generation;
static uint32_t                      ps2_idle_start_ms;
static uint8_t                       ps2_neutral_rearm_count;

#define PS2_HEADING_CRITICAL_QUALITY_MASK                                                                              \
    (STATE_ESTIMATION_IMU_QUALITY_SPI_ERROR | STATE_ESTIMATION_IMU_QUALITY_INIT_FAILED                                 \
     | STATE_ESTIMATION_IMU_QUALITY_FIFO_OVERFLOW | STATE_ESTIMATION_IMU_QUALITY_TIMESTAMP_ERROR                       \
     | STATE_ESTIMATION_IMU_QUALITY_GYRO_SATURATION | STATE_ESTIMATION_IMU_QUALITY_ATTITUDE_INVALID                    \
     | STATE_ESTIMATION_IMU_QUALITY_PROFILE_MISMATCH)

static void Ps2ControlService_CopyState(ps2_control_service_state_t *dst, const ps2_control_service_state_t *src)
{
    uint32_t primask;

    if (dst == 0 || src == 0)
    {
        return;
    }

    primask = PlatformCritical_Enter();
    *dst    = *src;
    PlatformCritical_Exit(primask);
}

static float Ps2ControlService_NormalizeAxis(uint8_t raw)
{
    int32_t delta     = (int32_t)raw - teleoperation_config.axis_center;
    int32_t magnitude = (delta < 0) ? -delta : delta;

    if (magnitude <= teleoperation_config.axis_deadzone)
    {
        return 0.0f;
    }

    if (delta > 0)
    {
        return (float)(delta - teleoperation_config.axis_deadzone) / (float)(127 - teleoperation_config.axis_deadzone);
    }
    return (float)(delta + teleoperation_config.axis_deadzone) / (float)(128 - teleoperation_config.axis_deadzone);
}

static float Ps2ControlService_ClampFloat(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static float Ps2ControlService_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Ps2ControlService_MaxStickMagnitude(const ps2_controller_driver_sample_t *sample)
{
    float maximum = 0.0f;
    float axes[4];

    if (sample == 0)
    {
        return 0.0f;
    }
    axes[0] = Ps2ControlService_AbsFloat(Ps2ControlService_NormalizeAxis(sample->left_x));
    axes[1] = Ps2ControlService_AbsFloat(Ps2ControlService_NormalizeAxis(sample->left_y));
    axes[2] = Ps2ControlService_AbsFloat(Ps2ControlService_NormalizeAxis(sample->right_x));
    axes[3] = Ps2ControlService_AbsFloat(Ps2ControlService_NormalizeAxis(sample->right_y));
    for (uint8_t index = 0U; index < 4U; ++index)
    {
        if (axes[index] > maximum)
        {
            maximum = axes[index];
        }
    }
    return maximum;
}

static void Ps2ControlService_SubmitCommand(float linear_x, float angular_z, uint32_t input_generation)
{
    command_intent_t intent = {
        .kind                = (linear_x == 0.0f && angular_z == 0.0f) ? COMMAND_INTENT_NEUTRAL : COMMAND_INTENT_ACTIVE,
        .source              = COMMAND_SOURCE_PS2,
        .linear_x            = linear_x,
        .angular_z           = angular_z,
        .sample_time_ms      = PlatformTime_TaskNowMs(),
        .producer_generation = ps2_state.generation,
        .expected_revoke_generation = input_generation,
    };

    (void)CommandManagement_ApplyIntent(&intent);
}

static void Ps2ControlService_SubmitHeadingCommand(float angular_z)
{
    command_intent_t intent = {
        .kind                       = COMMAND_INTENT_ACTIVE,
        .source                     = COMMAND_SOURCE_PS2,
        .linear_x                   = 0.0f,
        .angular_z                  = angular_z,
        .sample_time_ms             = PlatformTime_TaskNowMs(),
        .producer_generation        = ps2_state.generation,
        .expected_revoke_generation = heading_motion_generation,
    };

    (void)CommandManagement_ApplyIntent(&intent);
}

static void Ps2ControlService_ApplySlotIntent(command_intent_kind_t kind, uint32_t input_generation)
{
    command_intent_t intent = {
        .kind                       = kind,
        .source                     = COMMAND_SOURCE_PS2,
        .sample_time_ms             = PlatformTime_TaskNowMs(),
        .producer_generation        = ps2_state.generation,
        .expected_revoke_generation = input_generation,
    };

    (void)CommandManagement_ApplyIntent(&intent);
}

static uint8_t Ps2ControlService_ManualInputActive(float linear_x, float angular_z)
{
    float abs_linear  = (linear_x < 0.0f) ? -linear_x : linear_x;
    float abs_angular = (angular_z < 0.0f) ? -angular_z : angular_z;

    return (abs_linear > teleoperation_config.manual_cancel_threshold
            || abs_angular > teleoperation_config.manual_cancel_threshold)
               ? 1U
               : 0U;
}

static void Ps2ControlService_ApplyDpad(const ps2_controller_driver_sample_t *sample, float *linear_x, float *angular_z)
{
    uint8_t up_pressed;
    uint8_t down_pressed;
    uint8_t left_pressed;
    uint8_t right_pressed;

    if (sample == 0 || linear_x == 0 || angular_z == 0)
    {
        return;
    }

    up_pressed    = ((sample->btn1 & PS2_DPAD_UP_MASK) != 0U) ? 1U : 0U;
    down_pressed  = ((sample->btn1 & PS2_DPAD_DOWN_MASK) != 0U) ? 1U : 0U;
    left_pressed  = ((sample->btn1 & PS2_DPAD_LEFT_MASK) != 0U) ? 1U : 0U;
    right_pressed = ((sample->btn1 & PS2_DPAD_RIGHT_MASK) != 0U) ? 1U : 0U;

    if (up_pressed != 0U && down_pressed == 0U)
    {
        *linear_x = teleoperation_config.dpad_linear_mps;
    }
    else if (down_pressed != 0U && up_pressed == 0U)
    {
        *linear_x = -teleoperation_config.dpad_linear_mps;
    }

    if (left_pressed != 0U && right_pressed == 0U)
    {
        *angular_z = teleoperation_config.dpad_angular_rps;
    }
    else if (right_pressed != 0U && left_pressed == 0U)
    {
        *angular_z = -teleoperation_config.dpad_angular_rps;
    }
}

static uint32_t Ps2ControlService_ImuGateFlags(const state_estimation_imu_status_t *imu_state, uint32_t now_ms)
{
    uint32_t flags = 0U;

    if (imu_state == 0 || imu_state->enabled == 0U || imu_state->online == 0U
        || imu_state->last_error != STATE_ESTIMATION_IMU_ERROR_NONE
        || imu_state->init_state != STATE_ESTIMATION_IMU_INIT_STATE_SAMPLING || imu_state->sample_count == 0UL)
    {
        flags |= TELEOPERATION_HEADING_GATE_IMU_OFFLINE;
    }
    if (imu_state == 0 || imu_state->gyro_calibrated == 0U || imu_state->filter_initialized == 0U)
    {
        flags |= TELEOPERATION_HEADING_GATE_IMU_UNCALIBRATED;
    }
    if (imu_state == 0 || (uint32_t)(now_ms - imu_state->last_update_ms) > teleoperation_config.heading_imu_fresh_ms)
    {
        flags |= TELEOPERATION_HEADING_GATE_IMU_STALE;
    }
    if (imu_state == 0 || (imu_state->quality_flags & PS2_HEADING_CRITICAL_QUALITY_MASK) != 0UL)
    {
        flags |= TELEOPERATION_HEADING_GATE_IMU_QUALITY;
    }
    return flags;
}

static uint8_t Ps2ControlService_ImuUsable(const state_estimation_imu_status_t *imu_state, uint32_t now_ms)
{
    return (Ps2ControlService_ImuGateFlags(imu_state, now_ms) == 0U) ? 1U : 0U;
}

static uint8_t Ps2ControlService_StartMacro(uint8_t                              pressed,
                                            const state_estimation_imu_status_t *imu_state,
                                            uint32_t                             imu_now_ms,
                                            uint32_t                             control_now_ms,
                                            uint32_t                             input_generation)
{
    float    target_deg = 0.0f;
    uint32_t timeout_ms = 0U;

    if (Ps2ControlService_ImuUsable(imu_state, imu_now_ms) == 0U
        || input_generation != CommandManagement_GetMotionRevokeGeneration())
    {
        return 0U;
    }
    if ((pressed & teleoperation_config.macro_l1_mask) != 0U)
    {
        target_deg     = 90.0f;
        timeout_ms     = teleoperation_config.heading_quarter_timeout_ms;
        heading_button = teleoperation_config.macro_l1_mask;
    }
    else if ((pressed & teleoperation_config.macro_r1_mask) != 0U)
    {
        target_deg     = -90.0f;
        timeout_ms     = teleoperation_config.heading_quarter_timeout_ms;
        heading_button = teleoperation_config.macro_r1_mask;
    }
    else if ((pressed & teleoperation_config.macro_l2_mask) != 0U)
    {
        target_deg     = 360.0f;
        timeout_ms     = teleoperation_config.heading_full_timeout_ms;
        heading_button = teleoperation_config.macro_l2_mask;
    }
    else if ((pressed & teleoperation_config.macro_r2_mask) != 0U)
    {
        target_deg     = -360.0f;
        timeout_ms     = teleoperation_config.heading_full_timeout_ms;
        heading_button = teleoperation_config.macro_r2_mask;
    }
    else
    {
        return 0U;
    }

    if (RelativeHeadingController_Start(&heading_control, target_deg, imu_state->yaw_deg, control_now_ms, timeout_ms)
        == 0U)
    {
        return 0U;
    }
    heading_motion_generation = input_generation;
    if (heading_motion_generation != CommandManagement_GetMotionRevokeGeneration())
    {
        RelativeHeadingController_Cancel(&heading_control, RELATIVE_YAW_END_SAFETY_STOP);
        return 0U;
    }
    return 1U;
}

uint8_t Teleoperation_ValidateConfig(const teleoperation_config_t *config)
{
    if (config == 0 || config->linear_max_mps <= 0.0f || config->angular_max_rps <= 0.0f || config->axis_deadzone < 0
        || config->axis_deadzone >= 127 || config->offline_fail_limit == 0U || config->heading_imu_fresh_ms == 0UL
        || config->idle_release_ms == 0UL)
    {
        return 0U;
    }
    return 1U;
}

uint8_t Teleoperation_Init(const teleoperation_config_t *config)
{
    if (Teleoperation_ValidateConfig(config) == 0U)
    {
        return 0U;
    }
    teleoperation_config      = *config;
    ps2_state                 = (ps2_control_service_state_t){0};
    ps2_state.left_x          = (uint8_t)teleoperation_config.axis_center;
    ps2_state.left_y          = (uint8_t)teleoperation_config.axis_center;
    ps2_state.right_x         = (uint8_t)teleoperation_config.axis_center;
    ps2_state.right_y         = (uint8_t)teleoperation_config.axis_center;
    last_btn2                 = 0U;
    consecutive_read_failures = 0U;
    ps2_neutral_rearm_count   = 0U;
    RelativeHeadingController_Init(&heading_control);
    heading_button            = 0U;
    heading_zero_pending      = 0U;
    heading_motion_generation = CommandManagement_GetMotionRevokeGeneration();

    Ps2ControllerDriver_Init();
    ps2_state.cmd_dat_swapped = 0U;
    ps2_state.generation      = 1UL;
    return 1U;
}

void Teleoperation_Update(uint8_t line_tracking_enabled, teleoperation_action_t *action)
{
    ps2_controller_driver_sample_t sample;
    ps2_control_service_state_t    next_state;
    state_estimation_imu_status_t  imu_state;
    float                          linear_x  = 0.0f;
    float                          angular_z = 0.0f;
    uint8_t                        pressed_btn2;
    uint8_t                        macro_pressed;
    uint8_t                        command_active;
    uint8_t                        manual_active;
    uint8_t                        input_revoked;
    uint32_t                       now_ms           = PlatformTime_TaskNowMs();
    uint32_t                       input_generation = CommandManagement_GetMotionRevokeGeneration();

    if (action != 0)
    {
        *action = (teleoperation_action_t){TELEOPERATION_ACTION_NONE, now_ms, input_generation, line_tracking_enabled};
    }

    if (Ps2ControllerDriver_ReadSample(&sample) == 0U)
    {
        Ps2ControlService_CopyState(&next_state, &ps2_state);
        next_state.line_tracking_enabled = line_tracking_enabled;
        next_state.sample_valid          = 0U;
        next_state.rx_fail_count++;
        if (consecutive_read_failures < teleoperation_config.offline_fail_limit)
        {
            consecutive_read_failures++;
        }
        if (consecutive_read_failures < teleoperation_config.offline_fail_limit)
        {
            next_state.generation++;
            Ps2ControlService_CopyState(&ps2_state, &next_state);
            return;
        }

        next_state.online        = 0U;
        next_state.drive_enabled = 0U;
        if (heading_control.active != 0U)
        {
            RelativeHeadingController_Cancel(&heading_control, RELATIVE_YAW_END_CONTROLLER_OFFLINE);
        }
        heading_button                     = 0U;
        heading_zero_pending               = 0U;
        next_state.macro_active            = 0U;
        next_state.macro_button            = 0U;
        next_state.heading_active          = 0U;
        next_state.heading_end_reason      = (uint8_t)heading_control.end_reason;
        next_state.heading_target_deg      = heading_control.target_delta_deg;
        next_state.heading_accumulated_deg = heading_control.accumulated_delta_deg;
        next_state.linear_x                = 0.0f;
        next_state.angular_z               = 0.0f;
        next_state.generation++;
        Ps2ControlService_CopyState(&ps2_state, &next_state);
        ps2_idle_start_ms = 0U;
        Ps2ControlService_ApplySlotIntent(COMMAND_INTENT_RELEASE, input_generation);
        return;
    }
    consecutive_read_failures = 0U;
    input_revoked             = (input_generation != CommandManagement_GetMotionRevokeGeneration()) ? 1U : 0U;
    (void)StateEstimation_GetImu(&imu_state);
    uint32_t imu_now_ms = PlatformTime_NowMs();

    linear_x  = -Ps2ControlService_NormalizeAxis(sample.left_y) * teleoperation_config.linear_max_mps;
    angular_z = -Ps2ControlService_NormalizeAxis(sample.right_x) * teleoperation_config.angular_max_rps;
    Ps2ControlService_ApplyDpad(&sample, &linear_x, &angular_z);
    pressed_btn2  = (uint8_t)(sample.btn2 & (uint8_t)~last_btn2);
    macro_pressed = (uint8_t)(pressed_btn2
                              & (teleoperation_config.macro_l1_mask | teleoperation_config.macro_r1_mask
                                 | teleoperation_config.macro_l2_mask | teleoperation_config.macro_r2_mask));
    last_btn2     = sample.btn2;

    /* 巡线模式切换：三角键上升沿触发 */
    if (input_revoked == 0U && (pressed_btn2 & teleoperation_config.line_toggle_mask) != 0U)
    {
        if (action != 0)
        {
            action->type = TELEOPERATION_ACTION_TOGGLE_LINE;
        }
    }

    /* 巡线标定：方块=地板，圆形=黑线（上升沿触发 */
    if (input_revoked == 0U)
    {
        if ((pressed_btn2 & teleoperation_config.linecal_floor_mask) != 0U)
        {
            if (action != 0 && action->type == TELEOPERATION_ACTION_NONE)
            {
                action->type = TELEOPERATION_ACTION_CALIBRATE_LINE_FLOOR;
            }
        }
        if ((pressed_btn2 & teleoperation_config.linecal_line_mask) != 0U)
        {
            if (action != 0 && action->type == TELEOPERATION_ACTION_NONE)
            {
                action->type = TELEOPERATION_ACTION_CALIBRATE_LINE_SURFACE;
            }
        }
    }

    linear_x  = Ps2ControlService_ClampFloat(linear_x, teleoperation_config.linear_max_mps);
    angular_z = Ps2ControlService_ClampFloat(angular_z, teleoperation_config.angular_max_rps);

    manual_active = Ps2ControlService_ManualInputActive(linear_x, angular_z);
    if (CommandManagement_IsMotionGateOpen() != 0U && manual_active == 0U && macro_pressed == 0U)
    {
        if (ps2_neutral_rearm_count < 4U)
        {
            ps2_neutral_rearm_count++;
        }
        if (ps2_neutral_rearm_count == 3U)
        {
            Ps2ControlService_ApplySlotIntent(COMMAND_INTENT_REARM, input_generation);
            ps2_neutral_rearm_count = 4U;
        }
    }
    else
    {
        ps2_neutral_rearm_count = 0U;
    }
    if (CommandManagement_IsMotionGateOpen() == 0U || input_revoked != 0U
        || (heading_control.active != 0U && heading_motion_generation != CommandManagement_GetMotionRevokeGeneration()))
    {
        if (heading_control.active != 0U)
        {
            RelativeHeadingController_Cancel(&heading_control, RELATIVE_YAW_END_SAFETY_STOP);
        }
        heading_button       = 0U;
        heading_zero_pending = 0U;
        linear_x             = 0.0f;
        angular_z            = 0.0f;
        command_active       = 0U;
    }
    else if (manual_active != 0U)
    {
        if (heading_control.active != 0U)
        {
            RelativeHeadingController_Cancel(&heading_control, RELATIVE_YAW_END_MANUAL_OVERRIDE);
        }
        heading_button       = 0U;
        heading_zero_pending = 0U;
        command_active       = 1U;
    }
    else
    {
        if (heading_control.active == 0U && heading_zero_pending == 0U)
        {
            uint8_t macro_started =
                Ps2ControlService_StartMacro(pressed_btn2, &imu_state, imu_now_ms, now_ms, input_generation);
            if (macro_started == 0U && macro_pressed != 0U
                && Ps2ControlService_ImuGateFlags(&imu_state, imu_now_ms) != 0U)
            {
                /* Keep retrying only while the physical macro button remains held. */
                last_btn2 = (uint8_t)(last_btn2 & (uint8_t)~macro_pressed);
            }
        }
        if (heading_control.active != 0U)
        {
            if (RelativeHeadingController_Update(&heading_control,
                                                 imu_state.yaw_deg,
                                                 imu_state.body_gyro_dps[2],
                                                 now_ms,
                                                 &angular_z)
                != 0U)
            {
                command_active = 1U;
            }
            else
            {
                heading_button       = 0U;
                heading_zero_pending = 1U;
                command_active       = 0U;
            }
        }
        else
        {
            command_active = 0U;
        }
    }

    if (command_active == 0U)
    {
        linear_x  = 0.0f;
        angular_z = 0.0f;
    }

    Ps2ControlService_CopyState(&next_state, &ps2_state);
    next_state.rx_ok_count++;
    next_state.online             = 1U;
    next_state.analog_mode        = Ps2ControllerDriver_IsAnalogMode(sample.mode);
    next_state.drive_enabled      = command_active;
    next_state.btn1               = sample.btn1;
    next_state.btn2               = sample.btn2;
    next_state.left_x             = sample.left_x;
    next_state.left_y             = sample.left_y;
    next_state.right_x            = sample.right_x;
    next_state.right_y            = sample.right_y;
    next_state.macro_active       = heading_control.active;
    next_state.macro_button       = heading_button;
    next_state.heading_active     = heading_control.active;
    next_state.heading_end_reason = (uint8_t)heading_control.end_reason;
    next_state.pressed_btn2       = pressed_btn2;
    next_state.sample_valid       = 1U;
    next_state.dpad_active =
        ((sample.btn1 & (PS2_DPAD_UP_MASK | PS2_DPAD_RIGHT_MASK | PS2_DPAD_DOWN_MASK | PS2_DPAD_LEFT_MASK)) != 0U) ? 1U
                                                                                                                   : 0U;
    next_state.macro_edge              = (macro_pressed != 0U) ? 1U : 0U;
    next_state.heading_gate_flags      = Ps2ControlService_ImuGateFlags(&imu_state, imu_now_ms);
    next_state.imu_age_ms              = (uint32_t)(imu_now_ms - imu_state.last_update_ms);
    next_state.heading_target_deg      = heading_control.target_delta_deg;
    next_state.heading_accumulated_deg = heading_control.accumulated_delta_deg;
    next_state.linear_x                = linear_x;
    next_state.angular_z               = angular_z;
    next_state.stick_max_normalized    = Ps2ControlService_MaxStickMagnitude(&sample);
    next_state.line_tracking_enabled   = line_tracking_enabled;
    next_state.generation++;
    Ps2ControlService_CopyState(&ps2_state, &next_state);

    if (command_active != 0U)
    {
        ps2_idle_start_ms = 0U;
        if (heading_control.active != 0U)
        {
            Ps2ControlService_SubmitHeadingCommand(angular_z);
        }
        else
        {
            Ps2ControlService_SubmitCommand(linear_x, angular_z, input_generation);
        }
        return;
    }
    if (heading_zero_pending != 0U)
    {
        heading_zero_pending = 0U;
        ps2_idle_start_ms    = 0U;
        Ps2ControlService_SubmitCommand(0.0f, 0.0f, input_generation);
        return;
    }
    /* PS2 is online but idle — after a grace period, release the slot so
     lower-priority sources (ESP12F) can take over. */
    if (ps2_idle_start_ms == 0U)
    {
        ps2_idle_start_ms = now_ms;
    }
    if ((uint32_t)(now_ms - ps2_idle_start_ms) >= teleoperation_config.idle_release_ms)
    {
        Ps2ControlService_ApplySlotIntent(COMMAND_INTENT_RELEASE, input_generation);
        return;
    }
    Ps2ControlService_SubmitCommand(0.0f, 0.0f, input_generation);
}

void Teleoperation_OnManualModeEntered(uint32_t motion_revoke_generation)
{
    if (heading_control.active != 0U)
    {
        heading_motion_generation = motion_revoke_generation;
    }
    ps2_neutral_rearm_count = 4U;
}

uint32_t Teleoperation_GetStatus(teleoperation_status_t *state)
{
    Ps2ControlService_CopyState(state, &ps2_state);
    return (state != 0) ? state->generation : 0UL;
}
