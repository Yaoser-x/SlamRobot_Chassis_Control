#include "ps2_control.h"

#include "chassis_config.h"
#include "cmsis_os2.h"
#include "control_manager.h"
#include "imu_bmi270.h"
#include "led_status.h"
#include "line_calibration.h"
#include "line_control.h"
#include "main.h"
#include "ps2_hw.h"
#include "relative_yaw_control.h"

#define PS2_DPAD_UP_MASK       0x10U
#define PS2_DPAD_RIGHT_MASK    0x20U
#define PS2_DPAD_DOWN_MASK     0x40U
#define PS2_DPAD_LEFT_MASK     0x80U
#define PS2_HEADING_MACRO_MASK (PS2_MACRO_L1_MASK | PS2_MACRO_R1_MASK | PS2_MACRO_L2_MASK | PS2_MACRO_R2_MASK)

static ps2_control_state_t    ps2_state;
static uint8_t                last_btn2;
static uint8_t                consecutive_read_failures;
static relative_yaw_control_t heading_control;
static uint8_t                heading_button;
static uint8_t                heading_zero_pending;
static uint32_t               heading_motion_generation;
static uint32_t               ps2_idle_start_ms;

#define PS2_HEADING_CRITICAL_QUALITY_MASK                                                                              \
    (IMU_BMI270_QUALITY_SPI_ERROR | IMU_BMI270_QUALITY_INIT_FAILED | IMU_BMI270_QUALITY_FIFO_OVERFLOW                  \
     | IMU_BMI270_QUALITY_TIMESTAMP_ERROR | IMU_BMI270_QUALITY_GYRO_SATURATION | IMU_BMI270_QUALITY_ATTITUDE_INVALID   \
     | IMU_BMI270_QUALITY_PROFILE_MISMATCH)

static void Ps2Control_CopyState(ps2_control_state_t *dst, const ps2_control_state_t *src)
{
    uint32_t primask;

    if (dst == 0 || src == 0)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *dst = *src;
    __set_PRIMASK(primask);
}

static float Ps2Control_NormalizeAxis(uint8_t raw)
{
    int32_t delta     = (int32_t)raw - PS2_AXIS_CENTER;
    int32_t magnitude = (delta < 0) ? -delta : delta;

    if (magnitude <= PS2_AXIS_DEADZONE)
    {
        return 0.0f;
    }

    if (delta > 0)
    {
        return (float)(delta - PS2_AXIS_DEADZONE) / (float)(127 - PS2_AXIS_DEADZONE);
    }
    return (float)(delta + PS2_AXIS_DEADZONE) / (float)(128 - PS2_AXIS_DEADZONE);
}

static float Ps2Control_ClampFloat(float value, float limit)
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

static void Ps2Control_SubmitCommand(float linear_x, float angular_z, uint32_t input_generation)
{
    chassis_cmd_t cmd = {
        .linear_x     = linear_x,
        .angular_z    = angular_z,
        .enable       = 1U,
        .source       = CONTROL_SOURCE_PS2,
        .timestamp_ms = osKernelGetTickCount(),
    };

    (void)ControlManager_SetCommandForGeneration(&cmd, input_generation);
}

static void Ps2Control_SubmitHeadingCommand(float angular_z)
{
    chassis_cmd_t cmd = {
        .linear_x     = 0.0f,
        .angular_z    = angular_z,
        .enable       = 1U,
        .source       = CONTROL_SOURCE_PS2,
        .timestamp_ms = osKernelGetTickCount(),
    };

    (void)ControlManager_SetCommandForGeneration(&cmd, heading_motion_generation);
}

static uint8_t Ps2Control_ManualInputActive(float linear_x, float angular_z)
{
    float abs_linear  = (linear_x < 0.0f) ? -linear_x : linear_x;
    float abs_angular = (angular_z < 0.0f) ? -angular_z : angular_z;

    return (abs_linear > PS2_MANUAL_CANCEL_THRESHOLD || abs_angular > PS2_MANUAL_CANCEL_THRESHOLD) ? 1U : 0U;
}

static void Ps2Control_ApplyDpad(const ps2_hw_sample_t *sample, float *linear_x, float *angular_z)
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
        *linear_x = PS2_DPAD_LINEAR_MPS;
    }
    else if (down_pressed != 0U && up_pressed == 0U)
    {
        *linear_x = -PS2_DPAD_LINEAR_MPS;
    }

    if (left_pressed != 0U && right_pressed == 0U)
    {
        *angular_z = PS2_DPAD_ANGULAR_RPS;
    }
    else if (right_pressed != 0U && left_pressed == 0U)
    {
        *angular_z = -PS2_DPAD_ANGULAR_RPS;
    }
}

static uint32_t Ps2Control_ImuGateFlags(const imu_bmi270_state_t *imu_state, uint32_t now_ms)
{
    uint32_t flags = 0U;

    if (imu_state == 0 || imu_state->enabled == 0U || imu_state->online == 0U
        || imu_state->last_error != IMU_BMI270_ERROR_NONE || imu_state->init_state != IMU_BMI270_INIT_STATE_SAMPLING
        || imu_state->sample_count == 0UL)
    {
        flags |= PS2_HEADING_GATE_IMU_OFFLINE;
    }
    if (imu_state == 0 || imu_state->gyro_calibrated == 0U || imu_state->filter_initialized == 0U)
    {
        flags |= PS2_HEADING_GATE_IMU_UNCALIBRATED;
    }
    if (imu_state == 0 || (uint32_t)(now_ms - imu_state->last_update_ms) > PS2_HEADING_IMU_FRESH_MS)
    {
        flags |= PS2_HEADING_GATE_IMU_STALE;
    }
    if (imu_state == 0 || (imu_state->quality_flags & PS2_HEADING_CRITICAL_QUALITY_MASK) != 0UL)
    {
        flags |= PS2_HEADING_GATE_IMU_QUALITY;
    }
    return flags;
}

static uint8_t Ps2Control_ImuUsable(const imu_bmi270_state_t *imu_state, uint32_t now_ms)
{
    return (Ps2Control_ImuGateFlags(imu_state, now_ms) == 0U) ? 1U : 0U;
}

static uint8_t Ps2Control_StartMacro(uint8_t                   pressed,
                                     const imu_bmi270_state_t *imu_state,
                                     uint32_t                  imu_now_ms,
                                     uint32_t                  control_now_ms,
                                     uint32_t                  input_generation)
{
    float    target_deg = 0.0f;
    uint32_t timeout_ms = 0U;

    if (Ps2Control_ImuUsable(imu_state, imu_now_ms) == 0U
        || input_generation != ControlManager_GetMotionRevokeGeneration())
    {
        return 0U;
    }
    if ((pressed & PS2_MACRO_L1_MASK) != 0U)
    {
        target_deg     = 90.0f;
        timeout_ms     = PS2_HEADING_QUARTER_TIMEOUT_MS;
        heading_button = PS2_MACRO_L1_MASK;
    }
    else if ((pressed & PS2_MACRO_R1_MASK) != 0U)
    {
        target_deg     = -90.0f;
        timeout_ms     = PS2_HEADING_QUARTER_TIMEOUT_MS;
        heading_button = PS2_MACRO_R1_MASK;
    }
    else if ((pressed & PS2_MACRO_L2_MASK) != 0U)
    {
        target_deg     = 360.0f;
        timeout_ms     = PS2_HEADING_FULL_TIMEOUT_MS;
        heading_button = PS2_MACRO_L2_MASK;
    }
    else if ((pressed & PS2_MACRO_R2_MASK) != 0U)
    {
        target_deg     = -360.0f;
        timeout_ms     = PS2_HEADING_FULL_TIMEOUT_MS;
        heading_button = PS2_MACRO_R2_MASK;
    }
    else
    {
        return 0U;
    }

    if (RelativeYawControl_Start(&heading_control, target_deg, imu_state->yaw_deg, control_now_ms, timeout_ms) == 0U)
    {
        return 0U;
    }
    heading_motion_generation = input_generation;
    if (heading_motion_generation != ControlManager_GetMotionRevokeGeneration())
    {
        RelativeYawControl_Cancel(&heading_control, RELATIVE_YAW_END_SAFETY_STOP);
        return 0U;
    }
    return 1U;
}

void Ps2Control_Init(void)
{
    ps2_state                 = (ps2_control_state_t){0};
    ps2_state.left_x          = PS2_AXIS_CENTER;
    ps2_state.left_y          = PS2_AXIS_CENTER;
    ps2_state.right_x         = PS2_AXIS_CENTER;
    ps2_state.right_y         = PS2_AXIS_CENTER;
    last_btn2                 = 0U;
    consecutive_read_failures = 0U;
    RelativeYawControl_Init(&heading_control);
    heading_button            = 0U;
    heading_zero_pending      = 0U;
    heading_motion_generation = ControlManager_GetMotionRevokeGeneration();

    Ps2Hw_Init();
    ps2_state.cmd_dat_swapped = 0U;
}

void Ps2Control_Update(void)
{
    ps2_hw_sample_t     sample;
    ps2_control_state_t next_state;
    imu_bmi270_state_t  imu_state;
    float               linear_x  = 0.0f;
    float               angular_z = 0.0f;
    uint8_t             pressed_btn2;
    uint8_t             macro_pressed;
    uint8_t             command_active;
    uint8_t             manual_active;
    uint8_t             input_revoked;
    uint32_t            now_ms           = osKernelGetTickCount();
    uint32_t            input_generation = ControlManager_GetMotionRevokeGeneration();

    if (Ps2Hw_ReadSample(&sample) == 0U)
    {
        ps2_state.rx_fail_count++;
        if (consecutive_read_failures < PS2_OFFLINE_FAIL_LIMIT)
        {
            consecutive_read_failures++;
        }
        if (consecutive_read_failures < PS2_OFFLINE_FAIL_LIMIT)
        {
            return;
        }

        Ps2Control_CopyState(&next_state, &ps2_state);
        next_state.online        = 0U;
        next_state.drive_enabled = 0U;
        if (heading_control.active != 0U)
        {
            RelativeYawControl_Cancel(&heading_control, RELATIVE_YAW_END_CONTROLLER_OFFLINE);
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
        Ps2Control_CopyState(&ps2_state, &next_state);
        ps2_idle_start_ms = 0U;
        ControlManager_ClearSource(CONTROL_SOURCE_PS2);
        return;
    }
    consecutive_read_failures = 0U;
    input_revoked             = (input_generation != ControlManager_GetMotionRevokeGeneration()) ? 1U : 0U;
    ps2_state.rx_ok_count++;
    ImuBmi270_GetState(&imu_state);
    uint32_t imu_now_ms = HAL_GetTick();

    linear_x  = -Ps2Control_NormalizeAxis(sample.left_y) * PS2_LINEAR_MAX_MPS;
    angular_z = -Ps2Control_NormalizeAxis(sample.right_x) * PS2_ANGULAR_MAX_RPS;
    Ps2Control_ApplyDpad(&sample, &linear_x, &angular_z);
    pressed_btn2  = (uint8_t)(sample.btn2 & (uint8_t)~last_btn2);
    macro_pressed = (uint8_t)(pressed_btn2 & PS2_HEADING_MACRO_MASK);
    last_btn2     = sample.btn2;

    /* 巡线模式切换：三角键上升沿触发 */
    if (input_revoked == 0U && (pressed_btn2 & PS2_LINE_TOGGLE_MASK) != 0U)
    {
        LineControl_Enable((LineControl_IsEnabled() == 0U) ? 1U : 0U);
    }

    /* 巡线标定：方块=地板，圆形=黑线（上升沿触发，不与其他操作冲突） */
    if (input_revoked == 0U)
    {
        line_calibration_t cal_state;

        LineControl_CalibrationGet(&cal_state);

        if ((pressed_btn2 & PS2_LINECAL_FLOOR_MASK) != 0U)
        {
            if (cal_state.collecting == 0U)
            {
                (void)LineControl_CalibrationBegin(LINE_CALIBRATION_SURFACE_FLOOR, 100U);
                LedStatus_SetMode(LED_STATUS_CAL_RUNNING);
            }
        }
        if ((pressed_btn2 & PS2_LINECAL_LINE_MASK) != 0U)
        {
            if (cal_state.collecting == 0U)
            {
                (void)LineControl_CalibrationBegin(LINE_CALIBRATION_SURFACE_LINE, 100U);
                LedStatus_SetMode(LED_STATUS_CAL_RUNNING);
            }
        }

        /* 采集完成后提示单面成功 / 双面完成自动 apply */
        {
            static uint8_t prev_ready_mask;

            if (cal_state.collecting == 0U && cal_state.ready_mask != prev_ready_mask)
            {
                if ((cal_state.ready_mask & 0x03U) == 0x03U)
                {
                    /* 双面采集完成 → 自动 apply */
                    if (LineControl_CalibrationApplyAndSave() != 0U)
                    {
                        LedStatus_SetMode(LED_STATUS_CAL_APPLIED);
                    }
                    else
                    {
                        /* 分离度不足，标定失败 — 快闪提示 */
                        LedStatus_SetMode(LED_STATUS_CAL_RUNNING);
                    }
                    /* 清标定状态，防止重复触发 apply */
                    LineControl_CalibrationCancel();
                    prev_ready_mask = 0U;
                }
                else
                {
                    /* 单面完成 */
                    LedStatus_SetMode(LED_STATUS_CAL_OK);
                    prev_ready_mask = cal_state.ready_mask;
                }
            }
        }
    }

    linear_x  = Ps2Control_ClampFloat(linear_x, PS2_LINEAR_MAX_MPS);
    angular_z = Ps2Control_ClampFloat(angular_z, PS2_ANGULAR_MAX_RPS);

    manual_active = Ps2Control_ManualInputActive(linear_x, angular_z);
    if (ControlManager_IsEmergencyStop() != 0U || ControlManager_IsFaultStop() != 0U
        || ControlManager_IsMaintenanceLocked() != 0U || input_revoked != 0U
        || (heading_control.active != 0U && heading_motion_generation != ControlManager_GetMotionRevokeGeneration()))
    {
        if (heading_control.active != 0U)
        {
            RelativeYawControl_Cancel(&heading_control, RELATIVE_YAW_END_SAFETY_STOP);
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
            RelativeYawControl_Cancel(&heading_control, RELATIVE_YAW_END_MANUAL_OVERRIDE);
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
                Ps2Control_StartMacro(pressed_btn2, &imu_state, imu_now_ms, now_ms, input_generation);
            if (macro_started == 0U && macro_pressed != 0U && Ps2Control_ImuGateFlags(&imu_state, imu_now_ms) != 0U)
            {
                /* Keep retrying only while the physical macro button remains held. */
                last_btn2 = (uint8_t)(last_btn2 & (uint8_t)~macro_pressed);
            }
        }
        if (heading_control.active != 0U)
        {
            if (RelativeYawControl_Update(&heading_control,
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

    Ps2Control_CopyState(&next_state, &ps2_state);
    next_state.online                  = 1U;
    next_state.analog_mode             = Ps2Hw_IsAnalogMode(sample.mode);
    next_state.drive_enabled           = command_active;
    next_state.btn1                    = sample.btn1;
    next_state.btn2                    = sample.btn2;
    next_state.left_x                  = sample.left_x;
    next_state.left_y                  = sample.left_y;
    next_state.right_x                 = sample.right_x;
    next_state.right_y                 = sample.right_y;
    next_state.macro_active            = heading_control.active;
    next_state.macro_button            = heading_button;
    next_state.heading_active          = heading_control.active;
    next_state.heading_end_reason      = (uint8_t)heading_control.end_reason;
    next_state.pressed_btn2            = pressed_btn2;
    next_state.heading_gate_flags      = Ps2Control_ImuGateFlags(&imu_state, imu_now_ms);
    next_state.imu_age_ms              = (uint32_t)(imu_now_ms - imu_state.last_update_ms);
    next_state.heading_target_deg      = heading_control.target_delta_deg;
    next_state.heading_accumulated_deg = heading_control.accumulated_delta_deg;
    next_state.linear_x                = linear_x;
    next_state.angular_z               = angular_z;
    next_state.line_tracking_enabled   = LineControl_IsEnabled();
    Ps2Control_CopyState(&ps2_state, &next_state);

    if (command_active != 0U)
    {
        ps2_idle_start_ms = 0U;
        if (heading_control.active != 0U)
        {
            Ps2Control_SubmitHeadingCommand(angular_z);
        }
        else
        {
            Ps2Control_SubmitCommand(linear_x, angular_z, input_generation);
        }
        return;
    }
    if (heading_zero_pending != 0U)
    {
        heading_zero_pending = 0U;
        ps2_idle_start_ms    = 0U;
        Ps2Control_SubmitCommand(0.0f, 0.0f, input_generation);
        return;
    }
    if (LineControl_IsEnabled() != 0U)
    {
        ps2_idle_start_ms = 0U;
        ControlManager_ClearSource(CONTROL_SOURCE_PS2);
        return;
    }
    /* PS2 is online but idle — after a grace period, release the slot so
     lower-priority sources (ESP12F) can take over. */
    if (ps2_idle_start_ms == 0U)
    {
        ps2_idle_start_ms = now_ms;
    }
    if ((uint32_t)(now_ms - ps2_idle_start_ms) >= PS2_IDLE_RELEASE_MS)
    {
        ControlManager_ClearSource(CONTROL_SOURCE_PS2);
        return;
    }
    Ps2Control_SubmitCommand(0.0f, 0.0f, input_generation);
}

void Ps2Control_GetState(ps2_control_state_t *state)
{
    Ps2Control_CopyState(state, &ps2_state);
}
