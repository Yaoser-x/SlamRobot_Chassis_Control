#include "chassis_service.h"
#include "chassis_feedback_guard.h"
#include "chassis_output_service.h"
#include "chassis_param_sync.h"
#include "chassis_snapshot.h"
#include "chassis_speed_loop.h"
#include "chassis_target_planner.h"
#include "chassis_test_mode.h"
#include "platform_critical.h"
#include "platform_time.h"
#include "bsp_config.h"

#include "adc_monitor.h"

#include "control_config.h"

#include "chassis_layout.h"

#include "chassis_kinematics.h"

#include "control_service.h"

#include "current_guard.h"

#include "encoder_driver.h"

#include "motor_driver.h"

#include "param_service.h"

#include "safety_service.h"

#include "imu_bmi270.h"

static chassis_service_snapshot_t chassis_state;
static uint32_t                   last_control_step_ms;
static uint8_t                    control_dt_initialized;
static volatile uint8_t           control_step_active;
static chassis_param_sync_t       param_sync;
static chassis_target_planner_t   target_planner;
static chassis_speed_loop_t       speed_loop;
static chassis_feedback_guard_t   feedback_guard;
static chassis_test_mode_t        test_mode;

static float ChassisService_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int8_t ChassisService_TargetSign(float value)
{
    if (value > CHASSIS_PID_DIRECTION_EPSILON_MPS)
    {
        return 1;
    }
    if (value < -CHASSIS_PID_DIRECTION_EPSILON_MPS)
    {
        return -1;
    }
    return 0;
}

static void ChassisService_ResolveSideTargetsWithParams(float                linear_x,
                                                        float                angular_z,
                                                        const param_model_t *params,
                                                        float               *left_mps,
                                                        float               *right_mps)
{
    ChassisMath_ResolveDifferentialTargets(linear_x, angular_z, params->track_width_m, left_mps, right_mps);
}

void ChassisService_ResolveSideTargets(float linear_x, float angular_z, float *left_mps, float *right_mps)
{
    param_model_t params;

    (void)ParamService_GetSnapshot(&params);
    ChassisService_ResolveSideTargetsWithParams(linear_x, angular_z, &params, left_mps, right_mps);
}

static void ChassisService_ResetRamps(void)
{
    ChassisTargetPlanner_Reset(&target_planner);
    chassis_state.straight_active                 = 0U;
    chassis_state.straight_direction              = 0;
    chassis_state.straight_transition_distance_m  = 0.0f;
    chassis_state.straight_in_transition          = 0U;
    chassis_state.straight_trim_mps               = 0.0f;
    chassis_state.straight_wheel_correction_mps   = 0.0f;
    chassis_state.straight_heading_error_deg      = 0.0f;
    chassis_state.straight_heading_integral_deg_s = 0.0f;
    chassis_state.straight_heading_correction_mps = 0.0f;
    chassis_state.straight_total_correction_mps   = 0.0f;
    chassis_state.straight_heading_degraded       = 0U;
    chassis_state.straight_derated                = 0U;
    chassis_state.pwm_saturated                   = 0U;
    chassis_state.control_source                  = CONTROL_SOURCE_NONE;
}

static void ChassisService_ResetPidTargets(void)
{
    ChassisSpeedLoop_ResetTargets(&speed_loop);
    ChassisFeedbackGuard_Reset(&feedback_guard, &chassis_state);
}

static uint8_t ChassisService_RefreshRuntimeParams(void)
{
    if (ChassisParamSync_Refresh(&param_sync, speed_loop.pid_motor) == 0U)
    {
        return 0U;
    }
    ChassisService_ResetRamps();
    ChassisService_ResetPidTargets();
    return 1U;
}

void ChassisService_CancelTestMode(void)
{
    ChassisTestMode_Cancel(&test_mode);
}

static void ChassisService_StopOutput(void)
{
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        chassis_state.motor_target_mps[i]    = 0.0f;
        chassis_state.motor_requested_mps[i] = 0.0f;
        chassis_state.motor_error_mps[i]     = 0.0f;
        chassis_state.motor_pid_active[i]    = 0U;
        chassis_state.motor_feedback_lost[i] = 0U;
        ChassisOutputService_SetMotor(&chassis_state, (motor_id_t)i, 0);
    }
    ChassisService_ResetRamps();
    ChassisService_ResetPidTargets();
    chassis_state.output_enabled = ChassisOutputService_AnyActive(&chassis_state);
    ChassisSnapshot_SyncSides(&chassis_state);
}

void ChassisService_Init(void)
{
    MotorDriver_Init();
    MotorDriver_SetSpeedGetter(EncoderDriver_GetMotorSpeedMps);
    ControlService_Init();
    CurrentGuard_Init();
    chassis_state = (chassis_service_snapshot_t){0};
    ChassisParamSync_Init(&param_sync);
    ChassisTargetPlanner_Init(&target_planner);
    ChassisSpeedLoop_Init(&speed_loop);
    ChassisFeedbackGuard_Init(&feedback_guard);
    ChassisTestMode_Init(&test_mode);
    last_control_step_ms   = 0U;
    control_dt_initialized = 0U;
    control_step_active    = 0U;
    (void)ChassisService_RefreshRuntimeParams();
    ChassisService_ResetRamps();
    ChassisService_ResetPidTargets();
    MotorDriver_StopAll(MOTOR_STOP_LOW_SIDE_BRAKE);
}

static void ChassisService_StepImpl(uint32_t now_ms)
{
    chassis_cmd_t                cmd;
    encoder_state_t              encoder_state;
    uint8_t                      valid_cmd;
    chassis_test_mode_snapshot_t test_snapshot;
    float                        dt_s;

    (void)ChassisService_RefreshRuntimeParams();
    MotorDriver_UpdateFaults();
    if (MotorDriver_HasFault() != 0U)
    {
        ControlService_SetFaultStop(1U);
    }
    EncoderDriver_GetState(&encoder_state);
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        chassis_state.motor_actual_mps[i]  = encoder_state.speed_mps[i];
        chassis_state.motor_speed_valid[i] = encoder_state.speed_valid[i];
    }
    ChassisSnapshot_SyncSides(&chassis_state);

    if (ControlService_IsMaintenanceLocked() != 0U || ControlService_IsEmergencyStop() != 0U
        || ControlService_IsFaultStop() != 0U)
    {
        ChassisService_CancelTestMode();
        ChassisService_EmergencyStop();
        return;
    }

    ChassisTestMode_GetSnapshot(&test_mode, now_ms, &test_snapshot);
    if (test_snapshot.expired != 0U)
    {
        ControlService_ClearCommand();
        ChassisService_StopOutput();
        return;
    }

    valid_cmd = ControlService_GetCommand(&cmd, now_ms);
    if (ChassisMath_ControlDt(now_ms, &last_control_step_ms, &control_dt_initialized, &dt_s) == 0U)
    {
        ChassisSpeedLoop_Reset(&speed_loop);
        ChassisService_StopOutput();
        return;
    }

    {
        adc_monitor_state_t adc_state;

        AdcMonitor_GetState(&adc_state);
        if (adc_state.current_zero_valid == 0U)
        {
            ChassisService_StopOutput();
            return;
        }
    }

    if (test_snapshot.open_loop_active != 0U)
    {
        ChassisTestMode_ApplyOpenLoop(&test_snapshot, &chassis_state, &speed_loop);
        chassis_state.output_enabled = ChassisOutputService_AnyActive(&chassis_state);
        ChassisService_ResetRamps();
        ChassisSnapshot_SetSideTargets(&chassis_state, 0.0f, 0.0f, 1U);
        ChassisSnapshot_SyncSides(&chassis_state);
        return;
    }

    if (test_snapshot.raw_input_active != 0U)
    {
        ChassisTestMode_ApplyRaw(&test_snapshot, &chassis_state, &speed_loop);
        ChassisService_ResetRamps();
        ChassisSnapshot_SetSideTargets(&chassis_state, 0.0f, 0.0f, 1U);
        chassis_state.output_enabled = ChassisOutputService_AnyActive(&chassis_state);
        ChassisSnapshot_SyncSides(&chassis_state);
        return;
    }

    if (valid_cmd != 0U)
    {
        motor_driver_state_t            motor_state;
        chassis_target_planner_input_t  planner_input = {0};
        chassis_target_planner_result_t planner_result;

        if (ChassisLayout_HasBothSides() == 0U)
        {
            ControlService_ClearCommand();
            ChassisService_StopOutput();
            return;
        }

        planner_input.now_ms                = now_ms;
        planner_input.dt_s                  = dt_s;
        planner_input.command               = &cmd;
        planner_input.params                = &param_sync.params;
        planner_input.motion_generation     = ControlService_GetMotionRevokeGeneration();
        planner_input.actual_left_mps       = chassis_state.left_actual_mps;
        planner_input.actual_right_mps      = chassis_state.right_actual_mps;
        planner_input.left_speed_valid      = chassis_state.left_speed_valid;
        planner_input.right_speed_valid     = chassis_state.right_speed_valid;
        planner_input.left_output_permille  = chassis_state.left_output_permille;
        planner_input.right_output_permille = chassis_state.right_output_permille;
        planner_input.left_current_limited  = chassis_state.left_current_limited;
        planner_input.right_current_limited = chassis_state.right_current_limited;
        if (ChassisService_AbsFloat(cmd.angular_z) <= 0.0001f && ChassisService_AbsFloat(cmd.linear_x) > 0.001f)
        {
            imu_bmi270_state_t imu_state;

            ImuBmi270_GetState(&imu_state);
            planner_input.imu_valid  = (imu_state.online != 0U && imu_state.gyro_calibrated != 0U
                                       && (uint32_t)(now_ms - imu_state.last_update_ms) <= 100U
                                       && (imu_state.quality_flags
                                           & (IMU_BMI270_QUALITY_SPI_ERROR | IMU_BMI270_QUALITY_TIMESTAMP_ERROR
                                              | IMU_BMI270_QUALITY_GYRO_SATURATION | IMU_BMI270_QUALITY_INIT_FAILED
                                              | IMU_BMI270_QUALITY_PROFILE_MISMATCH))
                                              == 0U)
                                           ? 1U
                                           : 0U;
            planner_input.gyro_z_dps = imu_state.gyro_corrected_dps[2];
        }
        ChassisTargetPlanner_Step(&target_planner, &planner_input, &planner_result);
        ChassisSnapshot_SetSideTargets(&chassis_state,
                                       planner_result.requested_left_mps,
                                       planner_result.requested_right_mps,
                                       1U);

        ChassisSnapshot_ApplyPlannerResult(&chassis_state, &planner_result, cmd.source);
        ChassisSnapshot_SetSideTargets(&chassis_state,
                                       planner_result.target_left_mps,
                                       planner_result.target_right_mps,
                                       0U);

        MotorDriver_GetState(&motor_state);
        if (ChassisFeedbackGuard_DetectFault(&feedback_guard, now_ms, &chassis_state, &encoder_state, &motor_state)
            != 0U)
        {
            SafetyService_LatchEncoderFeedbackFault();
            ChassisService_EmergencyStop();
            return;
        }

        if (planner_result.requested_left_mps == 0.0f && planner_result.requested_right_mps == 0.0f)
        {
            ChassisSpeedLoop_Reset(&speed_loop);
            ChassisService_ResetPidTargets();
            for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
            {
                ChassisOutputService_SetMotor(&chassis_state, (motor_id_t)i, 0);
            }
            chassis_state.output_enabled = ChassisOutputService_AnyActive(&chassis_state);
            ChassisSnapshot_SyncSides(&chassis_state);
            return;
        }

        for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
        {
            int16_t permille;
            int16_t base_permille;
            int8_t  actuator_limit_direction = 0;
            if (ChassisLayout_MotorEnabled((motor_id_t)i) == 0U)
            {
                ChassisSpeedLoop_ResetMotor(&speed_loop, (motor_id_t)i);
                chassis_state.motor_pid_active[i]    = 0U;
                chassis_state.motor_feedback_lost[i] = 0U;
                chassis_state.motor_error_mps[i]     = 0.0f;
                ChassisOutputService_SetMotor(&chassis_state, (motor_id_t)i, 0);
                continue;
            }
            base_permille = ChassisOutputService_MpsToPermille(chassis_state.motor_target_mps[i]);
            if (chassis_state.motor_current_limited[i] != 0U
                || ChassisService_AbsFloat((float)motor_state.applied_pwm[i])
                       < ChassisService_AbsFloat((float)motor_state.requested_pwm[i]))
            {
                actuator_limit_direction = ChassisService_TargetSign(chassis_state.motor_target_mps[i]);
            }
            if (CHASSIS_PID_ENABLED != 0U)
            {
                chassis_speed_loop_result_t speed_result;
                uint8_t                     feedback_usable = ChassisFeedbackGuard_CheckUsable(&feedback_guard,
                                                                           &chassis_state,
                                                                           (motor_id_t)i,
                                                                           chassis_state.motor_target_mps[i],
                                                                           chassis_state.motor_actual_mps[i],
                                                                           encoder_state.speed_valid[i]);
                chassis_state.motor_speed_valid[i]          = feedback_usable;
                speed_result                                = ChassisSpeedLoop_StepMotor(&speed_loop,
                                                          (motor_id_t)i,
                                                          chassis_state.motor_target_mps[i],
                                                          chassis_state.motor_actual_mps[i],
                                                          feedback_usable,
                                                          dt_s,
                                                          actuator_limit_direction,
                                                          base_permille,
                                                          motor_state.phase[i]);
                permille                                    = speed_result.permille;
                chassis_state.motor_pid_active[i]           = speed_result.pid_active;
                chassis_state.motor_error_mps[i]            = speed_result.error_mps;
            }
            else
            {
                chassis_state.motor_pid_active[i]    = 0U;
                chassis_state.motor_feedback_lost[i] = 0U;
                chassis_state.motor_error_mps[i]     = 0.0f;
                permille                             = base_permille;
            }
            ChassisOutputService_SetMotor(&chassis_state, (motor_id_t)i, permille);
        }
        chassis_state.output_enabled = 1U;
        ChassisSnapshot_SyncSides(&chassis_state);
        chassis_state.pwm_saturated =
            (uint8_t)(ChassisService_AbsFloat((float)chassis_state.left_output_permille) >= 850.0f
                      || ChassisService_AbsFloat((float)chassis_state.right_output_permille) >= 850.0f);
    }
    else
    {
        ChassisService_StopOutput();
    }
}

void ChassisService_Step(uint32_t now_ms)
{
    uint32_t primask    = PlatformCritical_Enter();
    control_step_active = 1U;
    PlatformCritical_Exit(primask);
    ChassisService_StepImpl(now_ms);
    primask             = PlatformCritical_Enter();
    control_step_active = 0U;
    PlatformCritical_Exit(primask);
}

uint8_t ChassisService_IsStepActive(void)
{
    uint8_t  active;
    uint32_t primask = PlatformCritical_Enter();
    active           = control_step_active;
    PlatformCritical_Exit(primask);
    return active;
}

void ChassisService_EmergencyStop(void)
{
    ChassisSpeedLoop_Reset(&speed_loop);
    ChassisService_CancelTestMode();
    ChassisService_ResetRamps();
    ChassisService_ResetPidTargets();
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        chassis_state.motor_target_mps[i]      = 0.0f;
        chassis_state.motor_requested_mps[i]   = 0.0f;
        chassis_state.motor_error_mps[i]       = 0.0f;
        chassis_state.motor_output_permille[i] = 0;
        chassis_state.motor_current_limited[i] = 0U;
        chassis_state.motor_pid_active[i]      = 0U;
        chassis_state.motor_feedback_lost[i]   = 0U;
    }
    chassis_state.output_enabled = 0U;
    MotorDriver_StopAll(MOTOR_STOP_LOW_SIDE_BRAKE);
    ChassisSnapshot_SyncSides(&chassis_state);
}

void ChassisService_OpenLoopTest(int16_t left_permille, int16_t right_permille)
{
    ChassisTestMode_SetOpenLoop(&test_mode, left_permille, right_permille);
}

void ChassisService_RawInputTest(int16_t left_forward_permille,
                                 int16_t left_reverse_permille,
                                 int16_t right_forward_permille,
                                 int16_t right_reverse_permille)
{
    ChassisTestMode_SetRawSides(&test_mode,
                                left_forward_permille,
                                left_reverse_permille,
                                right_forward_permille,
                                right_reverse_permille);
}

void ChassisService_RawMotorInputTest(motor_id_t motor, int16_t forward_permille, int16_t reverse_permille)
{
    ChassisTestMode_SetRawMotor(&test_mode, motor, forward_permille, reverse_permille);
}

void ChassisService_GetState(chassis_service_snapshot_t *state)
{
    if (state != 0)
    {
        uint32_t primask = PlatformCritical_Enter();
        *state           = chassis_state;
        PlatformCritical_Exit(primask);
    }
}
