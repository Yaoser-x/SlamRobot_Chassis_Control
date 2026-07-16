#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "power_adc_driver.h"
#include "bsp_config.h"
#include "control_config.h"
#include "bsp_config.h"
#include "chassis_service.h"
#include "command_management_service.h"
#include "control_service.h"
#include "wheel_encoder_driver.h"
#include "bmi270_driver.h"
#include "param_service.h"
#include "wheel_speed_pid_controller.h"
#include "safety_management_service.h"
#include "state_estimation_service.h"
#include "power_management_service.h"

static power_adc_driver_state_t fake_adc_state;
static wheel_encoder_state_t    fake_encoder_state;
static int16_t                  fake_signed_pwm[MOTOR_ID_COUNT];
static uint8_t                  fake_fault_stop;
static uint8_t                  fake_primask;
static uint8_t                  fake_maintenance_lock;
static uint32_t                 fake_tick_ms;
static uint8_t                  fake_command_valid;
static chassis_cmd_t            fake_command;
static motor_driver_state_t     fake_motor_state;
static uint32_t                 fake_encoder_fault_latch_count;
static bmi270_driver_state_t    fake_imu_state;
static uint32_t                 fake_motion_generation;

uint32_t __get_PRIMASK(void)
{
    return fake_primask;
}

void __disable_irq(void)
{
    fake_primask = 1U;
}

void __set_PRIMASK(uint32_t primask)
{
    fake_primask = primask;
}

uint32_t HAL_GetTick(void)
{
    return fake_tick_ms;
}

uint32_t osKernelGetTickCount(void)
{
    return fake_tick_ms;
}

void Bmi270Driver_GetState(bmi270_driver_state_t *state)
{
    *state = fake_imu_state;
}

uint32_t StateEstimation_GetImu(state_estimation_imu_status_t *state)
{
    _Static_assert(sizeof(*state) == sizeof(fake_imu_state), "IMU status test fixture layout mismatch");
    memcpy(state, &fake_imu_state, sizeof(*state));
    return 1U;
}

void MotorDriver_Init(void)
{
}

void MotorDriver_SetSpeedGetter(motor_speed_getter_t getter)
{
    (void)getter;
}

void MotorDriver_SetPermille(motor_id_t motor, int16_t permille)
{
    fake_signed_pwm[motor]                = permille;
    fake_motor_state.requested_pwm[motor] = permille;
    fake_motor_state.applied_pwm[motor]   = permille;
    fake_motor_state.effective_pwm[motor] = permille;
}

void MotorDriver_SetDirectionConfig(const int8_t direction[MOTOR_ID_COUNT])
{
    (void)direction;
}

void MotorDriver_StopAll(motor_stop_mode_t mode)
{
    (void)mode;
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        fake_signed_pwm[i]                = 0;
        fake_motor_state.requested_pwm[i] = 0;
        fake_motor_state.applied_pwm[i]   = 0;
        fake_motor_state.effective_pwm[i] = 0;
    }
}

void MotorDriver_UpdateFaults(void)
{
}

uint8_t MotorDriver_HasFault(void)
{
    return 0U;
}

void MotorDriver_GetState(motor_driver_state_t *state)
{
    if (state != 0)
    {
        *state = fake_motor_state;
    }
}

void ControlService_Init(void)
{
    fake_fault_stop = 0U;
}

void ControlService_SetFaultStop(uint8_t enabled)
{
    fake_fault_stop = (enabled != 0U) ? 1U : 0U;
}

void SafetyManagement_SetFaultStop(uint8_t enabled)
{
    ControlService_SetFaultStop(enabled);
}

uint8_t ControlService_IsEmergencyStop(void)
{
    return 0U;
}

uint8_t ControlService_IsFaultStop(void)
{
    return fake_fault_stop;
}

uint8_t ControlService_IsMaintenanceLocked(void)
{
    return fake_maintenance_lock;
}

uint8_t SafetyManagement_IsMotionAllowed(void)
{
    return (fake_fault_stop == 0U && fake_maintenance_lock == 0U) ? 1U : 0U;
}

uint8_t SafetyManagement_BeginMaintenance(void)
{
    if (fake_maintenance_lock != 0U)
    {
        return 0U;
    }
    fake_maintenance_lock = 1U;
    return 1U;
}

void SafetyManagement_EndMaintenance(void)
{
    fake_maintenance_lock = 0U;
}

uint8_t ControlService_GetCommand(chassis_cmd_t *cmd, uint32_t now_ms)
{
    (void)now_ms;
    if (fake_command_valid != 0U)
    {
        *cmd = fake_command;
        return 1U;
    }
    return 0U;
}

void ControlService_ClearCommand(void)
{
    fake_command_valid = 0U;
}

void CommandManagement_ClearAll(void)
{
    ControlService_ClearCommand();
}

uint8_t CommandManagement_GetActive(command_velocity_t *command, uint32_t now_ms)
{
    chassis_cmd_t legacy;

    if (ControlService_GetCommand(&legacy, now_ms) == 0U)
    {
        return 0U;
    }
    *command = (command_velocity_t){
        .linear_x     = legacy.linear_x,
        .angular_z    = legacy.angular_z,
        .enable       = legacy.enable,
        .source       = (command_source_t)legacy.source,
        .timestamp_ms = legacy.timestamp_ms,
    };
    return 1U;
}

uint32_t ControlService_GetMotionRevokeGeneration(void)
{
    return fake_motion_generation;
}

uint32_t CommandManagement_GetMotionRevokeGeneration(void)
{
    return ControlService_GetMotionRevokeGeneration();
}

void SafetyManagement_LatchEncoderFeedbackFault(void)
{
    fake_encoder_fault_latch_count++;
    fake_fault_stop = 1U;
}

void WheelEncoderDriver_GetState(wheel_encoder_state_t *state)
{
    *state = fake_encoder_state;
}

uint32_t StateEstimation_GetWheel(state_estimation_wheel_status_t *state)
{
    _Static_assert(sizeof(*state) == sizeof(fake_encoder_state), "wheel status test fixture layout mismatch");
    memcpy(state, &fake_encoder_state, sizeof(*state));
    return 1U;
}

float WheelEncoderDriver_GetMotorSpeedMps(motor_id_t motor)
{
    if ((uint32_t)motor >= MOTOR_ID_COUNT)
    {
        return 0.0f;
    }
    return fake_encoder_state.speed_mps[(uint32_t)motor];
}

void PowerAdcDriver_GetState(power_adc_driver_state_t *state)
{
    *state = fake_adc_state;
}

uint32_t PowerManagement_GetStatus(power_management_status_t *state)
{
    _Static_assert(sizeof(*state) == sizeof(fake_adc_state), "power status test fixture layout mismatch");
    memcpy(state, &fake_adc_state, sizeof(*state));
    return 1U;
}

static void require_int(int condition, const char *message)
{
    if (condition == 0)
    {
        (void)printf("FAIL: %s\n", message);
        exit(1);
    }
}

static void reset_fake_chassis(void)
{
    fake_adc_state                 = (power_adc_driver_state_t){0};
    fake_encoder_state             = (wheel_encoder_state_t){0};
    fake_fault_stop                = 0U;
    fake_maintenance_lock          = 0U;
    fake_primask                   = 0U;
    fake_tick_ms                   = 0U;
    fake_command_valid             = 0U;
    fake_command                   = (chassis_cmd_t){0};
    fake_motor_state               = (motor_driver_state_t){0};
    fake_encoder_fault_latch_count = 0UL;
    fake_imu_state                 = (bmi270_driver_state_t){0};
    fake_motion_generation         = 0UL;
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        fake_signed_pwm[i]                = 0;
        fake_encoder_state.speed_valid[i] = 1U;
        fake_motor_state.phase[i]         = MOTOR_DRIVER_PHASE_RUN;
    }
    fake_encoder_state.speed_valid_all = 1U;
    fake_adc_state.current_valid       = 1U;
    fake_adc_state.current_zero_valid  = 1U;
    ParamService_SetDefaults();
    ChassisService_Init();
}

static void set_closed_loop_command(float linear_mps)
{
    fake_command = (chassis_cmd_t){
        .linear_x     = linear_mps,
        .angular_z    = 0.0f,
        .enable       = 1U,
        .source       = CONTROL_SOURCE_DEBUG,
        .timestamp_ms = 1000U,
    };
    fake_command_valid = 1U;
}

static void test_high_adc_current_does_not_throttle_pwm_output(void)
{
    chassis_service_snapshot_t state;

    reset_fake_chassis();
    fake_adc_state.current_a[MOTOR_ID_M2] = MOTOR_STALL_CURRENT_A + 1.0f;

    fake_tick_ms = 1000U;
    ChassisService_RawMotorInputTest(MOTOR_ID_M2, 50, 0);
    ChassisService_Step(1000U);
    ChassisService_GetState(&state);

    require_int(fake_signed_pwm[MOTOR_ID_M2] == 50, "M2 high ADC current forwards signed raw PWM target");
    require_int(state.motor_output_permille[MOTOR_ID_M2] == 50, "M2 high ADC current state reports applied output");
    require_int(state.motor_current_limited[MOTOR_ID_M2] == 0U, "M2 high ADC current does not report dynamic limit");
}

static void test_invalid_current_zero_blocks_test_outputs(void)
{
    reset_fake_chassis();
    fake_adc_state.current_zero_valid = 0U;

    fake_tick_ms = 1000U;
    ChassisService_RawMotorInputTest(MOTOR_ID_M2, 50, 0);
    ChassisService_Step(1000U);
    require_int(fake_signed_pwm[MOTOR_ID_M2] == 0, "raw output is blocked before current zero");

    ChassisService_OpenLoopTest(100, 100);
    ChassisService_Step(1010U);
    require_int(fake_signed_pwm[MOTOR_ID_M2] == 0, "open-loop output is blocked before current zero");
    require_int(fake_signed_pwm[MOTOR_ID_M3] == 0, "all open-loop outputs stay zero before current zero");
}

static void test_raw_test_mode_has_400ms_deadman(void)
{
    reset_fake_chassis();
    fake_tick_ms = 1000U;
    ChassisService_RawMotorInputTest(MOTOR_ID_M2, 50, 0);
    ChassisService_Step(1000U);
    ChassisService_Step(1010U);
    require_int(fake_signed_pwm[MOTOR_ID_M2] == 50, "raw output starts during lease");

    for (uint32_t now_ms = 1020U; now_ms <= 1400U; now_ms += 10U)
    {
        ChassisService_Step(now_ms);
    }
    require_int(fake_signed_pwm[MOTOR_ID_M2] == 50, "raw output remains at 400ms boundary");
    ChassisService_Step(1401U);
    require_int(fake_signed_pwm[MOTOR_ID_M2] == 0, "raw output stops after 400ms lease");
}

static void test_maintenance_lock_cancels_raw_before_output(void)
{
    reset_fake_chassis();
    fake_tick_ms = 2000U;
    ChassisService_RawMotorInputTest(MOTOR_ID_M2, 80, 0);
    ChassisService_Step(2000U);
    ChassisService_Step(2010U);
    require_int(fake_signed_pwm[MOTOR_ID_M2] == 80, "raw output active before maintenance");

    fake_maintenance_lock = 1U;
    ChassisService_Step(2020U);
    require_int(fake_signed_pwm[MOTOR_ID_M2] == 0, "maintenance lock stops raw output");
    fake_maintenance_lock = 0U;
    ChassisService_Step(2030U);
    require_int(fake_signed_pwm[MOTOR_ID_M2] == 0, "raw output does not resume after maintenance");
}

static void test_runtime_track_width_changes_side_targets(void)
{
    param_model_t params;
    float         left;
    float         right;

    reset_fake_chassis();
    (void)ParamService_GetSnapshot(&params);
    params.track_width_m = 0.200f;
    require_int(ParamService_Set(&params) != 0U, "first runtime track width accepted");
    ChassisService_ResolveSideTargets(0.0f, 1.0f, &left, &right);
    require_int(left < -0.099f && right > 0.099f, "first runtime track width applied");

    params.track_width_m = 0.400f;
    require_int(ParamService_Set(&params) != 0U, "second runtime track width accepted");
    ChassisService_ResolveSideTargets(0.0f, 1.0f, &left, &right);
    require_int(left < -0.199f && right > 0.199f, "updated runtime track width applied");
}

static void test_invalid_enabled_encoder_stops_whole_chassis_same_step(void)
{
    reset_fake_chassis();
    set_closed_loop_command(0.2f);
    fake_encoder_state.speed_valid[MOTOR_ID_M2] = 0U;
    ChassisService_Step(1000U);

    require_int(fake_encoder_fault_latch_count == 1UL, "invalid enabled encoder latches feedback fault");
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        require_int(fake_signed_pwm[i] == 0, "feedback fault stops every motor in same step");
    }
}

static void test_zero_motion_feedback_fault_after_150ms_run(void)
{
    reset_fake_chassis();
    set_closed_loop_command(0.2f);
    for (uint32_t now_ms = 1000U; now_ms < 1160U; now_ms += 10U)
    {
        ChassisService_Step(now_ms);
    }
    require_int(fake_encoder_fault_latch_count == 0UL, "zero motion does not fault before 150ms RUN window");
    ChassisService_Step(1160U);
    require_int(fake_encoder_fault_latch_count == 1UL, "zero motion faults at 150ms RUN window");
}

static void test_non_run_motor_phase_does_not_false_latch_feedback(void)
{
    reset_fake_chassis();
    set_closed_loop_command(0.2f);
    fake_motor_state.phase[MOTOR_ID_M2]         = MOTOR_DRIVER_PHASE_PH_SETTLE;
    fake_motor_state.phase[MOTOR_ID_M3]         = MOTOR_DRIVER_PHASE_REVERSE_BRAKE;
    fake_motor_state.effective_pwm[MOTOR_ID_M2] = 100;
    fake_motor_state.effective_pwm[MOTOR_ID_M3] = 100;
    for (uint32_t now_ms = 1000U; now_ms <= 1300U; now_ms += 10U)
    {
        ChassisService_Step(now_ms);
    }
    require_int(fake_encoder_fault_latch_count == 0UL, "non-RUN transition phases do not latch feedback fault");
}

static void test_disabled_encoder_invalid_is_ignored(void)
{
    reset_fake_chassis();
    set_closed_loop_command(0.2f);
    fake_encoder_state.speed_valid[MOTOR_ID_M1] = 0U;
    ChassisService_Step(1000U);
    require_int(fake_encoder_fault_latch_count == 0UL, "disabled encoder invalid is ignored");
}

static void test_all_remote_sources_share_straight_line_controller_and_imu_degrade(void)
{
    static const uint8_t sources[] = {
        CONTROL_SOURCE_UPPER,
        CONTROL_SOURCE_PS2,
        CONTROL_SOURCE_ESP12F,
        CONTROL_SOURCE_DEBUG,
    };
    for (uint8_t i = 0U; i < (uint8_t)(sizeof(sources) / sizeof(sources[0])); ++i)
    {
        param_model_t              params;
        chassis_service_snapshot_t state;

        reset_fake_chassis();
        (void)ParamService_GetSnapshot(&params);
        params.straight_wheel_coupling_gain  = 0.1f;
        params.straight_heading_kp           = 0.01f;
        params.straight_heading_hold_enabled = 1U;
        require_int(ParamService_Set(&params) != 0U, "straight parameters accepted");
        set_closed_loop_command(0.2f);
        fake_command.source = sources[i];
        ChassisService_Step(1000U);
        ChassisService_GetState(&state);
        require_int(state.straight_active != 0U, "zero-angular source enters shared compensation");
        require_int(state.straight_heading_degraded != 0U,
                    "invalid IMU degrades to wheel-only compensation in same cycle");
    }
}

static chassis_service_snapshot_t run_straight_with_imu(const bmi270_driver_state_t *imu)
{
    param_model_t              params;
    chassis_service_snapshot_t state;

    reset_fake_chassis();
    fake_imu_state = *imu;
    (void)ParamService_GetSnapshot(&params);
    params.straight_wheel_coupling_gain  = 0.1f;
    params.straight_heading_kp           = 0.01f;
    params.straight_heading_hold_enabled = 1U;
    require_int(ParamService_Set(&params) != 0U, "IMU straight parameters accepted");
    set_closed_loop_command(0.2f);
    ChassisService_Step(1000U);
    ChassisService_GetState(&state);
    return state;
}

static void test_straight_heading_gate_distinguishes_imu_failures(void)
{
    bmi270_driver_state_t      imu = {0};
    chassis_service_snapshot_t state;

    imu.online         = 1U;
    imu.last_update_ms = 1000U;
    state              = run_straight_with_imu(&imu);
    require_int(state.straight_heading_degraded != 0U, "uncalibrated IMU degrades");

    imu.gyro_calibrated = 1U;
    imu.last_update_ms  = 899U;
    state               = run_straight_with_imu(&imu);
    require_int(state.straight_heading_degraded != 0U, "stale IMU degrades");

    imu.last_update_ms = 1000U;
    imu.quality_flags  = STATE_ESTIMATION_IMU_QUALITY_ATTITUDE_INVALID;
    state              = run_straight_with_imu(&imu);
    require_int(state.straight_heading_degraded == 0U,
                "attitude-invalid IMU not degraded during settle — gyro rate damping works");

    imu.quality_flags = 0U;
    state             = run_straight_with_imu(&imu);
    require_int(state.straight_heading_degraded == 0U, "fresh calibrated quality-valid IMU accepted");
}

static void test_straight_integrates_gyro_without_settle_delay(void)
{
    param_model_t              params;
    chassis_service_snapshot_t state;

    reset_fake_chassis();
    (void)ParamService_GetSnapshot(&params);
    params.straight_wheel_coupling_gain          = 0.0f;
    params.straight_heading_kp                   = 0.01f;
    params.straight_heading_ki                   = 0.001f;
    params.straight_heading_integral_limit_deg_s = 10.0f;
    params.straight_heading_hold_enabled         = 1U;
    require_int(ParamService_Set(&params) != 0U, "gyro PI parameters accepted");
    fake_imu_state.online          = 1U;
    fake_imu_state.gyro_calibrated = 1U;
    fake_imu_state.last_update_ms  = 1000U;
    set_closed_loop_command(0.2f);
    ChassisService_Step(1000U);

    fake_imu_state.last_update_ms        = 1020U;
    fake_imu_state.gyro_corrected_dps[2] = 10.0f;
    ChassisService_Step(1020U);
    ChassisService_GetState(&state);
    require_int(state.straight_direction == 1, "straight direction is observable");
    require_int(state.straight_heading_error_deg < -0.19f,
                "gyro z integrates from straight start without settle delay");
    require_int(state.straight_heading_integral_deg_s < 0.0f, "heading PI integral is observable");
    require_int(state.straight_transition_distance_m >= 0.0f, "caster transition distance is observable");

    fake_command_valid = 0U;
    ChassisService_Step(1040U);
    ChassisService_GetState(&state);
    require_int(state.straight_active == 0U, "command loss clears straight state");
    require_int(state.control_source == CONTROL_SOURCE_NONE, "command loss clears logged source");
}

int main(void)
{
    test_high_adc_current_does_not_throttle_pwm_output();
    test_invalid_current_zero_blocks_test_outputs();
    test_raw_test_mode_has_400ms_deadman();
    test_maintenance_lock_cancels_raw_before_output();
    test_runtime_track_width_changes_side_targets();
    test_invalid_enabled_encoder_stops_whole_chassis_same_step();
    test_zero_motion_feedback_fault_after_150ms_run();
    test_non_run_motor_phase_does_not_false_latch_feedback();
    test_disabled_encoder_invalid_is_ignored();
    test_all_remote_sources_share_straight_line_controller_and_imu_degrade();
    test_straight_heading_gate_distinguishes_imu_failures();
    test_straight_integrates_gyro_without_settle_delay();
    return 0;
}
