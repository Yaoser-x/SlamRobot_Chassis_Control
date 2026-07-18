#include "motion_control_service.h"
#include "motion_control_maintenance.h"
#include "wheel_feedback_monitor.h"
#include "motor_output_coordinator.h"
#include "motion_parameter_sync.h"
#include "motion_status_builder.h"
#include "wheel_speed_control_loop.h"
#include "wheel_target_planner.h"
#include "motion_test_mode.h"
#include "platform_critical.h"
#include "platform_time.h"

#include "motor_hardware_layout.h"

#include "differential_drive_kinematics.h"

#include "command_management_service.h"

#include "motor_current_limiter.h"

#include "motor_driver.h"
#include "motion_maintenance.h"

#include "parameter_management_service.h"

#include "power_management_service.h"

#include "safety_management_service.h"

#include "state_estimation_service.h"

static motion_control_status_t    chassis_state;
static motion_control_status_t    published_state;
static motion_control_config_t    motion_config;
static uint32_t                   last_control_step_ms;
static uint8_t                    control_dt_initialized;
static uint8_t                    control_step_active;
static motion_parameter_sync_t    param_sync;
static wheel_target_planner_t     target_planner;
static wheel_speed_control_loop_t speed_loop;
static wheel_feedback_monitor_t   feedback_guard;
static motion_test_mode_t         test_mode;

static float ChassisService_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int8_t ChassisService_TargetSign(float value)
{
    if (value > motion_config.pid_direction_epsilon_mps)
    {
        return 1;
    }
    if (value < -motion_config.pid_direction_epsilon_mps)
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
    DifferentialDriveKinematics_ResolveDifferentialTargets(linear_x,
                                                           angular_z,
                                                           params->track_width_m,
                                                           left_mps,
                                                           right_mps);
}

void MotionControl_ResolveSideTargets(float linear_x, float angular_z, float *left_mps, float *right_mps)
{
    param_model_t params;

    (void)ParameterManagement_GetSnapshot(&params);
    ChassisService_ResolveSideTargetsWithParams(linear_x, angular_z, &params, left_mps, right_mps);
}

static void ChassisService_ResetRamps(void)
{
    WheelTargetPlanner_Reset(&target_planner);
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
    chassis_state.straight_out_of_range           = 0U;
    chassis_state.pwm_saturated                   = 0U;
    chassis_state.control_source                  = COMMAND_SOURCE_NONE;
}

static void ChassisService_ResetPidTargets(void)
{
    WheelSpeedControlLoop_ResetTargets(&speed_loop);
    WheelFeedbackMonitor_Reset(&feedback_guard, &chassis_state);
}

static void MotionControl_SyncDriverFacts(void)
{
    motor_driver_state_t motor_state;

    MotorDriver_GetState(&motor_state);
    chassis_state.motor_enabled_mask = 0U;
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        chassis_state.motor_effective_output_permille[index] = motor_state.effective_pwm[index];
        if (MotorHardwareLayout_MotorEnabled((motor_id_t)index) != 0U)
        {
            chassis_state.motor_enabled_mask |= (uint8_t)(1U << index);
        }
    }
}

static uint8_t ChassisService_RefreshRuntimeParams(void)
{
    if (MotionParameterSync_Refresh(&param_sync, speed_loop.pid_motor) == 0U)
    {
        return 0U;
    }
    ChassisService_ResetRamps();
    ChassisService_ResetPidTargets();
    return 1U;
}

void MotionControl_CancelTestMode(void)
{
    MotionTestMode_Cancel(&test_mode);
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
        MotorOutputCoordinator_SetMotor(&chassis_state, (motor_id_t)i, 0);
    }
    ChassisService_ResetRamps();
    ChassisService_ResetPidTargets();
    chassis_state.output_enabled = MotorOutputCoordinator_AnyActive(&chassis_state);
    MotionStatusBuilder_SyncSides(&chassis_state);
}

static float MotionControl_GetMotorSpeedMps(motor_id_t motor)
{
    state_estimation_wheel_status_t state;

    (void)StateEstimation_GetWheel(&state);
    return ((uint32_t)motor < MOTOR_ID_COUNT) ? state.speed_mps[motor] : 0.0f;
}

uint8_t MotionControl_ValidateConfig(const motion_control_config_t *config)
{
    if (config == 0 || config->max_linear_mps <= 0.0f || config->max_angular_rps <= 0.0f
        || config->open_loop_full_mps <= 0.0f || config->pid_correction_limit <= 0.0f
        || config->test_mode_lease_ms == 0UL || config->encoder_feedback_timeout_ms == 0UL
        || config->pid_feedback_loss_count == 0U)
    {
        return 0U;
    }
    return 1U;
}

uint8_t MotionControl_Init(const motion_control_config_t *config)
{
    if (MotionControl_ValidateConfig(config) == 0U)
    {
        return 0U;
    }
    motion_config = *config;
    MotorDriver_Init();
    MotorDriver_SetSpeedGetter(MotionControl_GetMotorSpeedMps);
    MotorCurrentLimiter_Init(config);
    MotorOutputCoordinator_Init(config);
    chassis_state = (motion_control_status_t){0};
    MotionParameterSync_Init(&param_sync, config);
    WheelTargetPlanner_Init(&target_planner, config);
    WheelSpeedControlLoop_Init(&speed_loop, config);
    WheelFeedbackMonitor_Init(&feedback_guard, config);
    MotionTestMode_Init(&test_mode, config);
    MotionMaintenance_Init(config->maintenance_max_speed_mps);
    last_control_step_ms   = 0U;
    control_dt_initialized = 0U;
    control_step_active    = 0U;
    (void)ChassisService_RefreshRuntimeParams();
    ChassisService_ResetRamps();
    ChassisService_ResetPidTargets();
    MotorDriver_StopAll(MOTOR_STOP_LOW_SIDE_BRAKE);
    MotionControl_SyncDriverFacts();
    chassis_state.generation = 1UL;
    published_state          = chassis_state;
    return 1U;
}

static void ChassisService_StepImpl(uint32_t now_ms)
{
    command_velocity_t              cmd;
    state_estimation_wheel_status_t encoder_state;
    uint8_t                         valid_cmd;
    motion_test_mode_snapshot_t     test_snapshot;
    float                           dt_s;

    (void)ChassisService_RefreshRuntimeParams();
    MotorDriver_UpdateFaults();
    if (MotorDriver_HasFault() != 0U)
    {
        SafetyManagement_SetFaultStop(1U);
    }
    (void)StateEstimation_GetWheel(&encoder_state);
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        chassis_state.motor_actual_mps[i]  = encoder_state.speed_mps[i];
        chassis_state.motor_speed_valid[i] = encoder_state.speed_valid[i];
    }
    MotionStatusBuilder_SyncSides(&chassis_state);

    if (SafetyManagement_IsMotionAllowed() == 0U)
    {
        MotionControl_CancelTestMode();
        MotionControl_EmergencyStop();
        return;
    }

    MotionTestMode_GetSnapshot(&test_mode, now_ms, &test_snapshot);
    if (test_snapshot.expired != 0U)
    {
        CommandManagement_ClearAll();
        ChassisService_StopOutput();
        return;
    }

    valid_cmd = CommandManagement_GetActive(&cmd, now_ms);
    if (DifferentialDriveKinematics_ControlDt(now_ms, &last_control_step_ms, &control_dt_initialized, &dt_s) == 0U)
    {
        WheelSpeedControlLoop_Reset(&speed_loop);
        ChassisService_StopOutput();
        return;
    }

    {
        power_management_status_t adc_state;

        (void)PowerManagement_GetStatus(&adc_state);
        if (adc_state.current_zero_valid == 0U)
        {
            ChassisService_StopOutput();
            return;
        }
    }

    if (test_snapshot.open_loop_active != 0U)
    {
        MotionTestMode_ApplyOpenLoop(&test_snapshot, &chassis_state, &speed_loop);
        chassis_state.output_enabled = MotorOutputCoordinator_AnyActive(&chassis_state);
        ChassisService_ResetRamps();
        MotionStatusBuilder_SetSideTargets(&chassis_state, 0.0f, 0.0f, 1U);
        MotionStatusBuilder_SyncSides(&chassis_state);
        return;
    }

    if (test_snapshot.raw_input_active != 0U)
    {
        MotionTestMode_ApplyRaw(&test_snapshot, &chassis_state, &speed_loop);
        ChassisService_ResetRamps();
        MotionStatusBuilder_SetSideTargets(&chassis_state, 0.0f, 0.0f, 1U);
        chassis_state.output_enabled = MotorOutputCoordinator_AnyActive(&chassis_state);
        MotionStatusBuilder_SyncSides(&chassis_state);
        return;
    }

    if (valid_cmd != 0U)
    {
        motor_driver_state_t          motor_state;
        wheel_target_planner_input_t  planner_input = {0};
        wheel_target_planner_result_t planner_result;

        if (MotorHardwareLayout_HasBothSides() == 0U)
        {
            CommandManagement_ClearAll();
            ChassisService_StopOutput();
            return;
        }

        planner_input.now_ms                = now_ms;
        planner_input.dt_s                  = dt_s;
        planner_input.command               = &cmd;
        planner_input.params                = &param_sync.params;
        planner_input.motion_generation     = CommandManagement_GetMotionRevokeGeneration();
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
            state_estimation_imu_status_t imu_state;

            (void)StateEstimation_GetImu(&imu_state);
            planner_input.imu_valid =
                (imu_state.online != 0U && imu_state.gyro_calibrated != 0U
                 && (uint32_t)(now_ms - imu_state.last_update_ms) <= 100U
                 && (imu_state.quality_flags
                     & (STATE_ESTIMATION_IMU_QUALITY_SPI_ERROR | STATE_ESTIMATION_IMU_QUALITY_TIMESTAMP_ERROR
                        | STATE_ESTIMATION_IMU_QUALITY_GYRO_SATURATION | STATE_ESTIMATION_IMU_QUALITY_INIT_FAILED
                        | STATE_ESTIMATION_IMU_QUALITY_PROFILE_MISMATCH))
                        == 0U)
                    ? 1U
                    : 0U;
            planner_input.gyro_z_dps = imu_state.gyro_corrected_dps[2];
        }
        WheelTargetPlanner_Step(&target_planner, &planner_input, &planner_result);
        MotionStatusBuilder_SetSideTargets(&chassis_state,
                                           planner_result.requested_left_mps,
                                           planner_result.requested_right_mps,
                                           1U);

        MotionStatusBuilder_ApplyPlannerResult(&chassis_state, &planner_result, cmd.source);
        MotionStatusBuilder_SetSideTargets(&chassis_state,
                                           planner_result.target_left_mps,
                                           planner_result.target_right_mps,
                                           0U);

        MotorDriver_GetState(&motor_state);
        if (WheelFeedbackMonitor_DetectFault(&feedback_guard, now_ms, &chassis_state, &encoder_state, &motor_state)
            != 0U)
        {
            SafetyManagement_LatchEncoderFeedbackFault();
            MotionControl_EmergencyStop();
            return;
        }

        if (planner_result.requested_left_mps == 0.0f && planner_result.requested_right_mps == 0.0f)
        {
            WheelSpeedControlLoop_Reset(&speed_loop);
            ChassisService_ResetPidTargets();
            for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
            {
                MotorOutputCoordinator_SetMotor(&chassis_state, (motor_id_t)i, 0);
            }
            chassis_state.output_enabled = MotorOutputCoordinator_AnyActive(&chassis_state);
            MotionStatusBuilder_SyncSides(&chassis_state);
            return;
        }

        for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
        {
            int16_t permille;
            int16_t base_permille;
            int8_t  actuator_limit_direction = 0;
            if (MotorHardwareLayout_MotorEnabled((motor_id_t)i) == 0U)
            {
                WheelSpeedControlLoop_ResetMotor(&speed_loop, (motor_id_t)i);
                chassis_state.motor_pid_active[i]    = 0U;
                chassis_state.motor_feedback_lost[i] = 0U;
                chassis_state.motor_error_mps[i]     = 0.0f;
                MotorOutputCoordinator_SetMotor(&chassis_state, (motor_id_t)i, 0);
                continue;
            }
            base_permille = MotorOutputCoordinator_MpsToPermille(chassis_state.motor_target_mps[i]);
            if (chassis_state.motor_current_limited[i] != 0U
                || ChassisService_AbsFloat((float)motor_state.applied_pwm[i])
                       < ChassisService_AbsFloat((float)motor_state.requested_pwm[i]))
            {
                actuator_limit_direction = ChassisService_TargetSign(chassis_state.motor_target_mps[i]);
            }
            if (motion_config.pid_enabled != 0U)
            {
                wheel_speed_control_loop_result_t speed_result;
                uint8_t                           feedback_usable = WheelFeedbackMonitor_CheckUsable(&feedback_guard,
                                                                           &chassis_state,
                                                                           (motor_id_t)i,
                                                                           chassis_state.motor_target_mps[i],
                                                                           chassis_state.motor_actual_mps[i],
                                                                           encoder_state.speed_valid[i]);
                chassis_state.motor_speed_valid[i]                = feedback_usable;
                speed_result                                      = WheelSpeedControlLoop_StepMotor(&speed_loop,
                                                               (motor_id_t)i,
                                                               chassis_state.motor_target_mps[i],
                                                               chassis_state.motor_actual_mps[i],
                                                               feedback_usable,
                                                               dt_s,
                                                               actuator_limit_direction,
                                                               base_permille,
                                                               motor_state.phase[i]);
                permille                                          = speed_result.permille;
                chassis_state.motor_pid_active[i]                 = speed_result.pid_active;
                chassis_state.motor_error_mps[i]                  = speed_result.error_mps;
            }
            else
            {
                chassis_state.motor_pid_active[i]    = 0U;
                chassis_state.motor_feedback_lost[i] = 0U;
                chassis_state.motor_error_mps[i]     = 0.0f;
                permille                             = base_permille;
            }
            MotorOutputCoordinator_SetMotor(&chassis_state, (motor_id_t)i, permille);
        }
        chassis_state.output_enabled = 1U;
        MotionStatusBuilder_SyncSides(&chassis_state);
        chassis_state.pwm_saturated =
            (uint8_t)(ChassisService_AbsFloat((float)chassis_state.left_output_permille) >= 850.0f
                      || ChassisService_AbsFloat((float)chassis_state.right_output_permille) >= 850.0f);
    }
    else
    {
        ChassisService_StopOutput();
    }
}

void MotionControl_Step(uint32_t now_ms)
{
    uint32_t primask    = PlatformCritical_Enter();
    control_step_active = 1U;
    PlatformCritical_Exit(primask);
    ChassisService_StepImpl(now_ms);
    MotionControl_SyncDriverFacts();
    primask = PlatformCritical_Enter();
    chassis_state.generation++;
    published_state     = chassis_state;
    control_step_active = 0U;
    PlatformCritical_Exit(primask);
}

uint8_t MotionControl_IsStepActive(void)
{
    uint8_t  active;
    uint32_t primask = PlatformCritical_Enter();
    active           = control_step_active;
    PlatformCritical_Exit(primask);
    return active;
}

void MotionControl_EmergencyStop(void)
{
    WheelSpeedControlLoop_Reset(&speed_loop);
    MotionControl_CancelTestMode();
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
    MotionControl_SyncDriverFacts();
    MotionStatusBuilder_SyncSides(&chassis_state);
    {
        uint32_t primask = PlatformCritical_Enter();
        chassis_state.generation++;
        published_state = chassis_state;
        PlatformCritical_Exit(primask);
    }
}

void MotionControl_OpenLoopTest(int16_t left_permille, int16_t right_permille)
{
    MotionTestMode_SetOpenLoop(&test_mode, left_permille, right_permille);
}

void MotionControl_RawInputTest(int16_t left_forward_permille,
                                int16_t left_reverse_permille,
                                int16_t right_forward_permille,
                                int16_t right_reverse_permille)
{
    MotionTestMode_SetRawSides(&test_mode,
                               left_forward_permille,
                               left_reverse_permille,
                               right_forward_permille,
                               right_reverse_permille);
}

void MotionControl_RawMotorInputTest(uint8_t motor, int16_t forward_permille, int16_t reverse_permille)
{
    if (motor < MOTOR_ID_COUNT)
    {
        MotionTestMode_SetRawMotor(&test_mode, (motor_id_t)motor, forward_permille, reverse_permille);
    }
}

uint32_t MotionControl_GetStatus(motion_control_status_t *state)
{
    if (state != 0)
    {
        uint32_t primask = PlatformCritical_Enter();
        *state           = published_state;
        PlatformCritical_Exit(primask);
        return state->generation;
    }
    return 0UL;
}
