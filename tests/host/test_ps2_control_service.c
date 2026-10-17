#include "ps2_control_service.h"

#include "command_management_service.h"
#include "control_config.h"
#include "bsp_config.h"
#include "control_service.h"
#include "status_led_driver.h"

#include "line_sensor_calibration.h"
#include "line_following_service.h"
#include "line_following_maintenance.h"
#include "ps2_controller_driver.h"
#include "relative_heading_controller.h"
#include "safety_management_service.h"
#include "state_estimation_service.h"
#include "teleoperation_service.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static uint32_t                       fake_primask;
static uint32_t                       fake_tick_ms;
static uint32_t                       fake_hal_tick_ms;
static uint8_t                        fake_read_ok;
static uint8_t                        fake_line_enabled;
static uint8_t                        fake_estop;
static uint8_t                        fake_fault_stop;
static uint8_t                        fake_maintenance;
static ps2_controller_driver_sample_t fake_sample;
static state_estimation_imu_status_t  fake_imu;
static chassis_cmd_t                  last_command;
static uint32_t                       set_command_count;
static uint32_t                       clear_source_count;
static uint32_t                       qualify_rearm_count;
static uint32_t                       fake_motion_revoke_generation;
static uint8_t                        fake_maintenance_crosses_on_query;
static line_sensor_calibration_t      fake_cal;
static uint8_t                        fake_cal_build_result;

static void require_int(int condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

uint32_t __get_PRIMASK(void)
{
    return fake_primask;
}
void __disable_irq(void)
{
    fake_primask = 1U;
}
void __set_PRIMASK(uint32_t value)
{
    fake_primask = value;
}
uint32_t osKernelGetTickCount(void)
{
    return fake_tick_ms;
}
uint32_t HAL_GetTick(void)
{
    return fake_hal_tick_ms;
}

void Ps2ControllerDriver_Init(void)
{
}
uint8_t Ps2ControllerDriver_ReadSample(ps2_controller_driver_sample_t *sample)
{
    if (fake_read_ok != 0U)
    {
        *sample = fake_sample;
    }
    return fake_read_ok;
}
uint8_t Ps2ControllerDriver_IsAnalogMode(uint8_t mode)
{
    return (mode == 0x73U) ? 1U : 0U;
}

control_command_result_t ControlService_SetCommand(const chassis_cmd_t *cmd)
{
    last_command = *cmd;
    set_command_count++;
    return CONTROL_COMMAND_ACCEPTED;
}
control_command_result_t ControlService_SetCommandForGeneration(const chassis_cmd_t *cmd, uint32_t expected_generation)
{
    if (expected_generation != fake_motion_revoke_generation)
    {
        return CONTROL_COMMAND_REJECTED;
    }
    return ControlService_SetCommand(cmd);
}
void ControlService_ClearSource(uint8_t source)
{
    require_int(source == CONTROL_SOURCE_PS2, "PS2 clears only its own source");
    clear_source_count++;
}
uint8_t ControlService_IsEmergencyStop(void)
{
    return fake_estop;
}
uint8_t ControlService_IsFaultStop(void)
{
    return fake_fault_stop;
}
uint8_t ControlService_IsMaintenanceLocked(void)
{
    if (fake_maintenance_crosses_on_query != 0U)
    {
        fake_maintenance_crosses_on_query = 0U;
        fake_motion_revoke_generation++;
    }
    return fake_maintenance;
}
uint32_t ControlService_GetMotionRevokeGeneration(void)
{
    return fake_motion_revoke_generation;
}

uint32_t CommandManagement_GetMotionRevokeGeneration(void)
{
    return ControlService_GetMotionRevokeGeneration();
}

uint8_t CommandManagement_IsMotionGateOpen(void)
{
    if (fake_maintenance_crosses_on_query != 0U)
    {
        fake_maintenance_crosses_on_query = 0U;
        fake_motion_revoke_generation++;
    }
    return (fake_estop == 0U && fake_fault_stop == 0U && fake_maintenance == 0U) ? 1U : 0U;
}

command_result_t CommandManagement_SetForGeneration(const command_velocity_t *command, uint32_t expected_generation)
{
    return ControlService_SetCommandForGeneration(command, expected_generation);
}

void CommandManagement_ClearSource(command_source_t source)
{
    ControlService_ClearSource((uint8_t)source);
}

command_apply_result_t CommandManagement_QualifyRearm(command_source_t source)
{
    qualify_rearm_count++;
    CommandManagement_ClearSource(source);
    return (command_apply_result_t){.outcome = COMMAND_OUTCOME_RELEASE_ACCEPTED, .source_cleared = 1U};
}

command_apply_result_t CommandManagement_ApplyIntent(const command_intent_t *intent)
{
    command_apply_result_t result = {COMMAND_OUTCOME_RELEASE_ACCEPTED, 0U, 1UL};

    if (intent->kind == COMMAND_INTENT_ACTIVE || intent->kind == COMMAND_INTENT_NEUTRAL)
    {
        command_velocity_t command = {
            .linear_x     = (intent->kind == COMMAND_INTENT_NEUTRAL) ? 0.0f : intent->linear_x,
            .angular_z    = (intent->kind == COMMAND_INTENT_NEUTRAL) ? 0.0f : intent->angular_z,
            .enable       = 1U,
            .source       = intent->source,
            .timestamp_ms = intent->sample_time_ms,
        };
        result.outcome = (CommandManagement_SetForGeneration(&command, intent->expected_revoke_generation)
                          == COMMAND_RESULT_ACCEPTED)
                             ? COMMAND_OUTCOME_ACTIVE_ACCEPTED
                             : COMMAND_OUTCOME_GENERATION_CONFLICT;
    }
    else if (intent->kind == COMMAND_INTENT_RELEASE)
    {
        CommandManagement_ClearSource(intent->source);
        result.source_cleared = 1U;
    }
    else if (intent->kind == COMMAND_INTENT_REARM)
    {
        result = CommandManagement_QualifyRearm(intent->source);
    }
    return result;
}

uint8_t SafetyManagement_IsMotionAllowed(void)
{
    return (ControlService_IsEmergencyStop() == 0U && ControlService_IsFaultStop() == 0U
            && ControlService_IsMaintenanceLocked() == 0U)
               ? 1U
               : 0U;
}

line_following_result_t LineFollowing_Enable(uint8_t enable)
{
    fake_line_enabled = (enable != 0U) ? 1U : 0U;
    return LINE_FOLLOWING_RESULT_APPLIED;
}
uint8_t LineFollowing_IsEnabled(void)
{
    return fake_line_enabled;
}
void LineFollowing_CalibrationGet(line_sensor_calibration_t *cal)
{
    if (cal)
        *cal = fake_cal;
}
uint8_t LineFollowing_RequestCalibration(line_sensor_calibration_surface_t surface, uint16_t samples)
{
    (void)surface;
    (void)samples;
    fake_cal.collecting = 1U;
    return 1U;
}
void LineFollowing_CalibrationCancel(void)
{
    fake_cal = (line_sensor_calibration_t){0};
}
line_calibration_apply_result_t LineFollowing_ApplyCalibration(void)
{
    return (fake_cal_build_result != 0U) ? LINE_CALIBRATION_APPLY_OK : LINE_CALIBRATION_APPLY_INCOMPLETE;
}
void StatusLedDriver_SetMode(status_led_driver_mode_t mode)
{
    (void)mode;
}
uint32_t StateEstimation_GetImu(state_estimation_imu_status_t *state)
{
    *state = fake_imu;
    return 1UL;
}
uint32_t StateEstimation_GetImuSampleSnapshot(state_estimation_imu_status_t *state)
{
    *state = fake_imu;
    return fake_imu.sample_count;
}

static void reset_fake(void)
{
    fake_primask      = 0U;
    fake_tick_ms      = 1000U;
    fake_hal_tick_ms  = fake_tick_ms;
    fake_read_ok      = 1U;
    fake_line_enabled = 0U;
    fake_estop        = 0U;
    fake_fault_stop   = 0U;
    fake_maintenance  = 0U;
    fake_sample       = (ps2_controller_driver_sample_t){
              .mode    = 0x73U,
              .right_x = PS2_AXIS_CENTER,
              .right_y = PS2_AXIS_CENTER,
              .left_x  = PS2_AXIS_CENTER,
              .left_y  = PS2_AXIS_CENTER,
    };
    fake_imu                          = (state_estimation_imu_status_t){0};
    fake_imu.enabled                  = 1U;
    fake_imu.online                   = 1U;
    fake_imu.last_error               = STATE_ESTIMATION_IMU_ERROR_NONE;
    fake_imu.init_state               = STATE_ESTIMATION_IMU_INIT_STATE_SAMPLING;
    fake_imu.sample_count             = 1UL;
    fake_imu.last_update_ms           = fake_hal_tick_ms;
    fake_imu.gyro_calibrated          = 1U;
    fake_imu.filter_initialized       = 1U;
    last_command                      = (chassis_cmd_t){0};
    set_command_count                 = 0UL;
    clear_source_count                = 0UL;
    qualify_rearm_count               = 0UL;
    fake_motion_revoke_generation     = 0UL;
    fake_maintenance_crosses_on_query = 0U;
    fake_cal                          = (line_sensor_calibration_t){0};
    fake_cal_build_result             = 0U;
    Ps2ControlService_Init();
}

static void test_heading_imu_freshness_uses_hal_tick_epoch(void)
{
    ps2_control_service_state_t state;

    reset_fake();
    fake_tick_ms            = 25U;
    fake_hal_tick_ms        = 5000U;
    fake_imu.last_update_ms = 4990U;
    fake_sample.btn2        = PS2_MACRO_L1_MASK;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_active != 0U, "fresh HAL-timestamped IMU starts heading despite RTOS epoch offset");
}

static void test_center_submits_zero_or_yields_to_line(void)
{
    reset_fake();
    Ps2ControlService_Update();
    require_int(set_command_count == 1UL, "centered online PS2 submits command when line is off");
    require_int(last_command.linear_x == 0.0f && last_command.angular_z == 0.0f, "centered online PS2 command is zero");

    /* Teleoperation no longer checks line state — PS2 continues normal idle behavior */
    fake_line_enabled = 1U;
    fake_tick_ms += 20U;
    Ps2ControlService_Update();
    require_int(set_command_count == 2UL, "centered PS2 continues submitting zero command regardless of line");

    fake_sample.right_x = 0U;
    fake_tick_ms += 20U;
    Ps2ControlService_Update();
    require_int(set_command_count >= 2UL, "PS2 idle continues normally");
}

static void test_neutral_rearm_is_qualified_once_per_observation(void)
{
    reset_fake();
    for (uint8_t cycle = 0U; cycle < 10U; ++cycle)
    {
        Ps2ControlService_Update();
        fake_tick_ms += 20U;
        fake_hal_tick_ms += 20U;
    }
    require_int(qualify_rearm_count == 1UL, "held neutral qualifies PS2 rearm only once");
}

static void test_manual_threshold_uses_normalized_axis_magnitude(void)
{
    ps2_control_service_state_t state;

    reset_fake();
    fake_sample.left_y = 90U;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.stick_max_normalized > 0.15f, "sample exceeds the normal MANUAL takeover threshold");
    require_int(state.drive_enabled != 0U && last_command.linear_x > 0.08f && last_command.linear_x < 0.10f,
                "normalized manual intent remains active below 0.12 m/s");
    require_int(last_command.angular_z == 0.0f, "linear stick does not fabricate angular intent");

    reset_fake();
    fake_sample.right_x = 150U;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.drive_enabled == 0U && last_command.angular_z == 0.0f,
                "small angular input uses the same normalized threshold as linear input");
}

static void test_slow_gear_scales_sticks_and_dpad(void)
{
    ps2_control_service_state_t state;
    const uint8_t               dpad_buttons[] = {0x10U, 0x40U, 0x80U, 0x20U};
    const float                 linear[]       = {0.1f, -0.1f, 0.0f, 0.0f};
    const float                 angular[]      = {0.0f, 0.0f, 1.0f, -1.0f};

    for (uint8_t index = 0U; index < 4U; ++index)
    {
        reset_fake();
        fake_sample.btn1 = PS2_SLOW_BUTTON_MASK;
        if (index < 2U)
        {
            fake_sample.left_y = (index == 0U) ? 0U : 255U;
        }
        else
        {
            fake_sample.right_x = (index == 2U) ? 0U : 255U;
        }
        Ps2ControlService_Update();
        Ps2ControlService_GetState(&state);
        require_int(state.slow_mode_active != 0U && state.drive_enabled != 0U,
                    "SELECT enables low gear without suppressing normalized manual intent");
        require_int(fabsf(last_command.linear_x - linear[index]) < 0.00001f
                        && fabsf(last_command.angular_z - angular[index]) < 0.00001f,
                    "low gear scales both directions of the driving sticks");
        require_int(state.linear_x == last_command.linear_x && state.angular_z == last_command.angular_z,
                    "status reports the scaled command submitted for arbitration");

        reset_fake();
        fake_sample.btn1 = (uint8_t)(PS2_SLOW_BUTTON_MASK | dpad_buttons[index]);
        Ps2ControlService_Update();
        require_int(fabsf(last_command.linear_x - linear[index]) < 0.00001f
                        && fabsf(last_command.angular_z - angular[index]) < 0.00001f,
                    "low gear applies to each D-pad direction");
    }
}

static void test_slow_gear_release_requires_neutral(void)
{
    ps2_control_service_state_t state;

    reset_fake();
    fake_sample.btn1 = PS2_SLOW_BUTTON_MASK;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.slow_mode_active != 0U && state.drive_enabled == 0U && state.stick_max_normalized == 0.0f
                    && last_command.angular_z == 0.0f,
                "SELECT alone does not create motion or stick takeover");

    fake_sample.right_x = 255U;
    Ps2ControlService_Update();
    fake_sample.btn1 = 0U;
    Ps2ControlService_Update();
    require_int(last_command.angular_z == -1.0f, "releasing SELECT while steering cannot increase speed");

    fake_sample.right_x = PS2_AXIS_CENTER;
    fake_sample.btn1    = 0x20U;
    Ps2ControlService_Update();
    require_int(last_command.angular_z == -1.0f, "held D-pad retains low gear after SELECT release");

    fake_sample.btn1 = 0U;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.slow_mode_active == 0U && last_command.angular_z == 0.0f,
                "neutral releases the low gear latch while stopped");
    fake_sample.right_x = 255U;
    Ps2ControlService_Update();
    require_int(last_command.angular_z == -PS2_ANGULAR_MAX_RPS, "a new ordinary input retains the existing speed cap");
}

static void test_slow_gear_keeps_safety_gates_and_macro_speed(void)
{
    ps2_control_service_state_t state;
    float                       ordinary_macro_speed;

    for (uint8_t gate = 0U; gate < 3U; ++gate)
    {
        reset_fake();
        fake_sample.btn1   = PS2_SLOW_BUTTON_MASK;
        fake_sample.left_y = 0U;
        fake_estop         = (gate == 0U) ? 1U : 0U;
        fake_fault_stop    = (gate == 1U) ? 1U : 0U;
        fake_maintenance   = (gate == 2U) ? 1U : 0U;
        Ps2ControlService_Update();
        Ps2ControlService_GetState(&state);
        require_int(state.drive_enabled == 0U && last_command.linear_x == 0.0f && last_command.angular_z == 0.0f,
                    "low gear cannot bypass ESTOP, fault or maintenance gates");
    }

    reset_fake();
    fake_sample.btn1                  = PS2_SLOW_BUTTON_MASK;
    fake_sample.left_y                = 0U;
    fake_maintenance_crosses_on_query = 1U;
    Ps2ControlService_Update();
    require_int(set_command_count == 0UL, "low gear cannot revive an input across a revoke generation");

    reset_fake();
    fake_sample.btn1   = PS2_SLOW_BUTTON_MASK;
    fake_sample.left_y = 0U;
    Ps2ControlService_Update();
    fake_read_ok = 0U;
    for (uint8_t failure = 0U; failure < PS2_OFFLINE_FAIL_LIMIT; ++failure)
    {
        Ps2ControlService_Update();
    }
    Ps2ControlService_GetState(&state);
    require_int(state.online == 0U && state.drive_enabled == 0U && state.slow_mode_active == 0U
                    && state.linear_x == 0.0f && clear_source_count != 0UL,
                "confirmed disconnect releases the low gear command and state");

    reset_fake();
    fake_sample.btn2 = PS2_MACRO_L1_MASK;
    Ps2ControlService_Update();
    ordinary_macro_speed = last_command.angular_z;
    reset_fake();
    fake_sample.btn1 = PS2_SLOW_BUTTON_MASK;
    fake_sample.btn2 = PS2_MACRO_L1_MASK;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_active != 0U && ordinary_macro_speed > 0.0f
                    && last_command.angular_z == ordinary_macro_speed,
                "SELECT does not change the separate heading controller or its timeout model");
}

static void test_slow_gear_config_validation_and_opt_out(void)
{
    teleoperation_config_t config = {
        .linear_max_mps             = 0.5f,
        .angular_max_rps            = 5.0f,
        .dpad_linear_mps            = 0.5f,
        .dpad_angular_rps           = 5.0f,
        .manual_cancel_threshold    = 0.12f,
        .heading_quarter_timeout_ms = 6000U,
        .heading_full_timeout_ms    = 20000U,
        .heading_imu_fresh_ms       = 50U,
        .idle_release_ms            = 2000U,
        .axis_center                = 128,
        .axis_deadzone              = 18,
        .offline_fail_limit         = 3U,
    };
    const float invalid_scales[] = {NAN, INFINITY, -0.1f, 1.1f};

    reset_fake();
    require_int(Teleoperation_Init(&config) != 0U, "zero-initialized extension preserves ordinary mapping");
    fake_sample.btn1    = PS2_SLOW_BUTTON_MASK;
    fake_sample.right_x = 255U;
    Ps2ControlService_Update();
    require_int(last_command.angular_z == -PS2_ANGULAR_MAX_RPS, "disabled slow gear ignores SELECT");

    config.slow_button_mask = PS2_SLOW_BUTTON_MASK;
    require_int(Teleoperation_ValidateConfig(&config) == 0U, "enabled slow gear requires a positive scale");
    for (uint8_t index = 0U; index < sizeof(invalid_scales) / sizeof(invalid_scales[0]); ++index)
    {
        config.slow_speed_scale = invalid_scales[index];
        require_int(Teleoperation_Init(&config) == 0U, "invalid slow gear scale is rejected before initialization");
    }
    config.slow_speed_scale = PS2_SLOW_SPEED_SCALE;
    require_int(Teleoperation_ValidateConfig(&config) != 0U, "default low gear configuration is valid");
    config.linear_max_mps = NAN;
    require_int(Teleoperation_ValidateConfig(&config) == 0U,
                "normalized threshold requires a finite positive speed cap");
}

static void test_heading_button_mapping(void)
{
    ps2_control_service_state_t state;

    reset_fake();
    fake_sample.btn2 = PS2_MACRO_L1_MASK;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_active != 0U && state.heading_target_deg == 90.0f, "L1 starts +90 relative heading");

    reset_fake();
    fake_sample.btn2 = PS2_MACRO_R1_MASK;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_target_deg == -90.0f, "R1 starts -90 relative heading");

    reset_fake();
    fake_sample.btn2 = PS2_MACRO_L2_MASK;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_target_deg == 360.0f, "L2 starts +360 relative heading");

    reset_fake();
    fake_sample.btn2 = PS2_MACRO_R2_MASK;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_target_deg == -360.0f, "R2 starts -360 relative heading");
}

static void test_imu_gate_and_runtime_cancel(void)
{
    ps2_control_service_state_t state;

    /* 未校准 IMU 拒绝启动 heading（ImuUsable 严格检查） */
    reset_fake();
    fake_imu.gyro_calibrated = 0U;
    fake_sample.btn2         = PS2_MACRO_L1_MASK;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_active == 0U, "uncalibrated IMU rejects heading macro");

    /* An active macro must stop when its heading feedback becomes invalid. */
    reset_fake();
    fake_sample.btn2 = PS2_MACRO_L1_MASK;
    Ps2ControlService_Update();
    fake_sample.btn2       = 0U;
    fake_imu.quality_flags = STATE_ESTIMATION_IMU_QUALITY_SPI_ERROR;
    fake_imu.online        = 0U;
    fake_tick_ms += 20U;
    fake_imu.last_update_ms = fake_tick_ms;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_active == 0U && state.heading_end_reason == RELATIVE_YAW_END_IMU_INVALID,
                "IMU error/offline cancels active heading");
    require_int(last_command.angular_z == 0.0f, "invalid IMU submits neutral in the same cycle");
}

static void test_heading_integrates_each_fresh_imu_sample_once(void)
{
    ps2_control_service_state_t state;
    reset_fake();
    fake_sample.btn2          = PS2_MACRO_L1_MASK;
    fake_imu.body_gyro_dps[2] = 90.0f;
    Ps2ControlService_Update();
    fake_sample.btn2 = 0U;
    fake_tick_ms += 20U;
    fake_hal_tick_ms += 20U;
    fake_imu.sample_count++;
    fake_imu.last_update_ms = fake_hal_tick_ms;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(fabsf(state.heading_accumulated_deg - 1.8f) < 0.001f,
                "fresh gyro sample integrates its sample interval");
    fake_tick_ms += 20U;
    fake_hal_tick_ms += 20U;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_active != 0U && fabsf(state.heading_accumulated_deg - 1.8f) < 0.001f,
                "duplicate fresh snapshot does not fabricate heading progress");
    fake_tick_ms += 40U;
    fake_hal_tick_ms += 40U;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_active == 0U && state.heading_end_reason == RELATIVE_YAW_END_IMU_INVALID
                    && last_command.angular_z == 0.0f,
                "frozen sample cancels macro at the configured freshness bound");
    fake_imu.sample_count++;
    fake_imu.last_update_ms = fake_hal_tick_ms;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_active == 0U, "IMU recovery without a new button edge does not restart macro");
}

static void test_held_macro_retries_after_transient_imu_gate(void)
{
    ps2_control_service_state_t state;

    reset_fake();
    fake_imu.last_update_ms = fake_hal_tick_ms - 51U;
    fake_sample.btn2        = PS2_MACRO_L1_MASK;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_active == 0U, "stale IMU initially rejects held macro");

    fake_tick_ms += 20U;
    fake_hal_tick_ms += 20U;
    fake_imu.last_update_ms = fake_hal_tick_ms;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_active != 0U && state.heading_target_deg == 90.0f,
                "held macro starts when transient IMU gate clears");
}

static void test_released_macro_does_not_start_after_imu_gate_clears(void)
{
    ps2_control_service_state_t state;

    reset_fake();
    fake_imu.last_update_ms = fake_hal_tick_ms - 51U;
    fake_sample.btn2        = PS2_MACRO_L1_MASK;
    Ps2ControlService_Update();

    fake_sample.btn2 = 0U;
    fake_tick_ms += 20U;
    fake_hal_tick_ms += 20U;
    fake_imu.last_update_ms = fake_hal_tick_ms;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_active == 0U, "released macro cannot start after transient IMU gate clears");
}

static void test_offline_and_safety_stop_cancel_heading(void)
{
    ps2_control_service_state_t state;

    reset_fake();
    fake_sample.btn2 = PS2_MACRO_L1_MASK;
    Ps2ControlService_Update();
    fake_read_ok = 0U;
    Ps2ControlService_Update();
    Ps2ControlService_Update();
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.online == 0U && state.heading_active == 0U, "three failed reads cancel heading and mark offline");
    require_int(clear_source_count != 0UL, "offline PS2 clears its source");

    reset_fake();
    fake_sample.btn2 = PS2_MACRO_L1_MASK;
    Ps2ControlService_Update();
    fake_sample.btn2 = 0U;
    fake_fault_stop  = 1U;
    fake_tick_ms += 20U;
    fake_imu.last_update_ms = fake_tick_ms;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_active == 0U && state.heading_end_reason == RELATIVE_YAW_END_SAFETY_STOP,
                "fault stop cancels heading macro");
}

static void test_revoke_generation_cancels_heading_after_short_safety_event(void)
{
    ps2_control_service_state_t state;

    reset_fake();
    fake_sample.btn2 = PS2_MACRO_L1_MASK;
    Ps2ControlService_Update();
    fake_sample.btn2 = 0U;
    fake_motion_revoke_generation++;
    fake_tick_ms += 20U;
    fake_imu.last_update_ms = fake_tick_ms;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_active == 0U && state.heading_end_reason == RELATIVE_YAW_END_SAFETY_STOP,
                "motion revoke generation cancels an old heading macro");
    require_int(last_command.linear_x == 0.0f && last_command.angular_z == 0.0f,
                "revoked heading macro submits zero speed");
}

static void test_maintenance_rejects_new_heading_macro(void)
{
    ps2_control_service_state_t state;

    reset_fake();
    fake_maintenance = 1U;
    fake_sample.btn2 = PS2_MACRO_L1_MASK;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_active == 0U, "maintenance rejects a new heading macro");
    require_int(last_command.linear_x == 0.0f && last_command.angular_z == 0.0f,
                "maintenance-time macro input cannot create motion");
}

static void test_heading_uses_generation_from_input_sample(void)
{
    ps2_control_service_state_t state;

    reset_fake();
    fake_sample.btn2                  = PS2_MACRO_L1_MASK;
    fake_maintenance_crosses_on_query = 1U;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.heading_active == 0U, "maintenance crossing after input sample invalidates macro edge");
}

static void test_status_generation_is_monotonic_on_whole_publish(void)
{
    teleoperation_status_t before;
    teleoperation_status_t after;

    reset_fake();
    (void)Teleoperation_GetStatus(&before);
    {
        teleoperation_action_t a = {0};
        Teleoperation_Update(fake_line_enabled, &a);
    }
    (void)Teleoperation_GetStatus(&after);
    require_int(after.generation > before.generation, "teleoperation status generation advances on update");
    require_int(after.online != 0U && after.left_x == fake_sample.left_x && after.right_x == fake_sample.right_x,
                "teleoperation status publishes one complete PS2 frame");
}

static void test_line_state_is_an_input_fact_on_every_path(void)
{
    ps2_control_service_state_t state;

    reset_fake();
    fake_line_enabled = 1U;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.line_tracking_enabled != 0U, "successful PS2 cycle publishes App-provided line state");

    reset_fake();
    fake_line_enabled = 1U;
    fake_read_ok      = 0U;
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.line_tracking_enabled != 0U, "short read failure publishes App-provided line state");
    Ps2ControlService_Update();
    Ps2ControlService_Update();
    Ps2ControlService_GetState(&state);
    require_int(state.online == 0U && state.line_tracking_enabled != 0U,
                "offline transition preserves App-provided line state");
}

int main(void)
{
    test_center_submits_zero_or_yields_to_line();
    test_neutral_rearm_is_qualified_once_per_observation();
    test_manual_threshold_uses_normalized_axis_magnitude();
    test_slow_gear_scales_sticks_and_dpad();
    test_slow_gear_release_requires_neutral();
    test_slow_gear_keeps_safety_gates_and_macro_speed();
    test_slow_gear_config_validation_and_opt_out();
    test_heading_button_mapping();
    test_heading_imu_freshness_uses_hal_tick_epoch();
    test_imu_gate_and_runtime_cancel();
    test_heading_integrates_each_fresh_imu_sample_once();
    test_held_macro_retries_after_transient_imu_gate();
    test_released_macro_does_not_start_after_imu_gate_clears();
    test_offline_and_safety_stop_cancel_heading();
    test_revoke_generation_cancels_heading_after_short_safety_event();
    test_maintenance_rejects_new_heading_macro();
    test_heading_uses_generation_from_input_sample();
    test_status_generation_is_monotonic_on_whole_publish();
    test_line_state_is_an_input_fact_on_every_path();
    (void)printf("PASS: PS2 control host tests\n");
    return 0;
}
