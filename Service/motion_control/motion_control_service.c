#include "motion_control_service.h"
#include "motion_control_maintenance.h"
#include "wheel_feedback_monitor.h"
#include "motor_output_coordinator.h"
#include "motion_parameter_sync.h"
#include "motion_status_builder.h"
#include "wheel_speed_control_loop.h"
#include "wheel_target_planner.h"
#include "motion_test_mode.h"
#include "safety_motion_permit_guard.h"
#include "platform_critical.h"
#include "platform_time.h"

#include "motor_hardware_layout.h"

#include "differential_drive_kinematics.h"

#include "motor_current_limiter.h"

#include "motor_driver.h"

static motion_control_status_t      chassis_state;
static motion_control_status_t      published_state;
static motion_control_config_t      motion_config;
static control_timing_t             control_timing;
static uint8_t                      control_step_active;
static motion_parameter_sync_t      param_sync;
static wheel_target_planner_t       target_planner;
static wheel_speed_control_loop_t   speed_loop;
static wheel_feedback_monitor_t     feedback_guard;
static motion_test_mode_t           test_mode;
static safety_motion_permit_guard_t safety_permit_guard;
static uint32_t                     motion_event_generation;
static float                        motion_wheel_speed_cache[MOTOR_ID_COUNT];

static void MotionControl_SetEvent(motion_control_event_t *event, uint32_t flags, uint32_t now_ms)
{
    if (event == 0 || flags == 0UL)
    {
        return;
    }
    if (event->flags == 0UL)
    {
        motion_event_generation++;
        if (motion_event_generation == 0UL)
        {
            motion_event_generation = 1UL;
        }
        event->occurred_at_ms = now_ms;
        event->generation     = motion_event_generation;
    }
    event->flags |= flags;
}

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

void MotionControl_ResolveSideTargetsWithParameters(float                linear_x,
                                                    float                angular_z,
                                                    const param_model_t *params,
                                                    float               *left_mps,
                                                    float               *right_mps)
{
    if (params == 0)
    {
        if (left_mps != 0)
        {
            *left_mps = 0.0f;
        }
        if (right_mps != 0)
        {
            *right_mps = 0.0f;
        }
        return;
    }
    ChassisService_ResolveSideTargetsWithParams(linear_x, angular_z, params, left_mps, right_mps);
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

static uint8_t ChassisService_ApplyRuntimeParams(const motion_parameter_fact_t *parameters)
{
    if (parameters == 0 || parameters->validity == 0UL
        || MotionParameterSync_Apply(&param_sync, &parameters->value, parameters->generation, speed_loop.pid_motor)
               == 0U)
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
        MotorOutputCoordinator_StopMotor(&chassis_state, (motor_id_t)i);
    }
    ChassisService_ResetRamps();
    ChassisService_ResetPidTargets();
    chassis_state.output_enabled = MotorOutputCoordinator_AnyActive(&chassis_state);
    MotionStatusBuilder_SyncSides(&chassis_state);
}

static float MotionControl_GetMotorSpeedMps(motor_id_t motor)
{
    return ((uint32_t)motor < MOTOR_ID_COUNT) ? motion_wheel_speed_cache[motor] : 0.0f;
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
    SafetyMotionPermitGuard_Init(&safety_permit_guard);
    control_timing          = (control_timing_t){0};
    control_step_active     = 0U;
    motion_event_generation = 0UL;
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        motion_wheel_speed_cache[index] = 0.0f;
    }
    ChassisService_ResetRamps();
    ChassisService_ResetPidTargets();
    MotorDriver_StopAll(MOTOR_STOP_LOW_SIDE_BRAKE);
    MotionControl_SyncDriverFacts();
    chassis_state.generation = 1UL;
    published_state          = chassis_state;
    return 1U;
}

static void ChassisService_StepImpl(const motion_control_input_t *input, motion_control_event_t *event)
{
    command_velocity_t              cmd;
    state_estimation_wheel_status_t encoder_state;
    uint8_t                         valid_cmd;
    motion_test_mode_snapshot_t     test_snapshot;
    float                           dt_s;
    uint32_t                        now_ms            = input->now_ms;
    uint32_t                        nominal_period_ms = input->nominal_period_ms;
    safety_motion_permit_result_t   permit_result;

    permit_result = SafetyMotionPermitGuard_Evaluate(&safety_permit_guard, &input->safety_permit.value, now_ms);
    if (permit_result != SAFETY_MOTION_PERMIT_ALLOW)
    {
        if (permit_result == SAFETY_MOTION_PERMIT_STALE || permit_result == SAFETY_MOTION_PERMIT_INVALID)
        {
            MotionControl_SetEvent(event, MOTION_EVENT_SAFETY_PERMIT_STALE, now_ms);
        }
        else if (permit_result == SAFETY_MOTION_PERMIT_GENERATION_CHANGED)
        {
            MotionControl_SetEvent(event, MOTION_EVENT_SAFETY_PERMIT_CHANGED, now_ms);
        }
        MotionControl_EmergencyStop();
        return;
    }

    (void)ChassisService_ApplyRuntimeParams(&input->parameters);
    MotorDriver_UpdateFaults();
    if (MotorDriver_HasFault() != 0U)
    {
        MotionControl_SetEvent(event, MOTION_EVENT_DRIVER_FAULT, now_ms);
        MotionControl_EmergencyStop();
        return;
    }
    encoder_state = input->wheel.value;
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        chassis_state.motor_actual_mps[i]  = encoder_state.speed_mps[i];
        chassis_state.motor_speed_valid[i] = encoder_state.speed_valid[i];
    }
    MotionStatusBuilder_SyncSides(&chassis_state);

    MotionTestMode_GetSnapshot(&test_mode, now_ms, &test_snapshot);
    if (input->normal_motion_allowed == 0U
        && ((test_snapshot.open_loop_active == 0U && test_snapshot.raw_input_active == 0U)
            || input->diagnostic_motion_allowed == 0U))
    {
        MotionControl_CancelTestMode();
        MotionControl_EmergencyStop();
        return;
    }

    if (test_snapshot.expired != 0U)
    {
        MotionControl_SetEvent(event, MOTION_EVENT_TEST_LEASE_EXPIRED | MOTION_EVENT_COMMAND_REVOKE, now_ms);
        ChassisService_StopOutput();
        return;
    }

    valid_cmd = (input->command.validity != 0UL) ? 1U : 0U;
    cmd       = input->command.value;
    control_timing_status_t timing_status =
        DifferentialDriveKinematics_EvaluateControlTiming(&control_timing, now_ms, nominal_period_ms);
    if (timing_status == CONTROL_TIMING_EARLY)
    {
        return;
    }
    if (timing_status == CONTROL_TIMING_FIRST || timing_status == CONTROL_TIMING_MISSED)
    {
        WheelSpeedControlLoop_Reset(&speed_loop);
        ChassisService_StopOutput();
        return;
    }
    dt_s = control_timing.dt_s;

    {
        if (input->power.validity == 0UL || input->power.value.current_zero_valid == 0U)
        {
            ChassisService_StopOutput();
            return;
        }
    }

    if (test_snapshot.open_loop_active != 0U)
    {
        MotionTestMode_ApplyOpenLoop(&test_snapshot,
                                     &chassis_state,
                                     &speed_loop,
                                     &input->power.value,
                                     &param_sync.params);
        chassis_state.output_enabled = MotorOutputCoordinator_AnyActive(&chassis_state);
        ChassisService_ResetRamps();
        MotionStatusBuilder_SetSideTargets(&chassis_state, 0.0f, 0.0f, 1U);
        MotionStatusBuilder_SyncSides(&chassis_state);
        return;
    }

    if (test_snapshot.raw_input_active != 0U)
    {
        MotionTestMode_ApplyRaw(&test_snapshot, &chassis_state, &speed_loop, &input->power.value, &param_sync.params);
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
            MotionControl_SetEvent(event, MOTION_EVENT_COMMAND_REVOKE, now_ms);
            ChassisService_StopOutput();
            return;
        }

        planner_input.now_ms                = now_ms;
        planner_input.dt_s                  = dt_s;
        planner_input.command               = &cmd;
        planner_input.params                = &param_sync.params;
        planner_input.motion_generation     = input->command.revoke_generation;
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
            const state_estimation_imu_status_t *imu_state = &input->imu.value;
            planner_input.imu_valid =
                (input->imu.validity != 0UL && imu_state->online != 0U && imu_state->gyro_calibrated != 0U
                 && (uint32_t)(now_ms - imu_state->last_update_ms) <= 100U
                 && (imu_state->quality_flags
                     & (STATE_ESTIMATION_IMU_QUALITY_SPI_ERROR | STATE_ESTIMATION_IMU_QUALITY_TIMESTAMP_ERROR
                        | STATE_ESTIMATION_IMU_QUALITY_GYRO_SATURATION | STATE_ESTIMATION_IMU_QUALITY_INIT_FAILED
                        | STATE_ESTIMATION_IMU_QUALITY_PROFILE_MISMATCH))
                        == 0U)
                    ? 1U
                    : 0U;
            planner_input.gyro_z_dps = imu_state->gyro_corrected_dps[2];
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
            MotionControl_SetEvent(event, MOTION_EVENT_ENCODER_FEEDBACK_LOST, now_ms);
            MotionControl_EmergencyStop();
            return;
        }

        if (planner_result.requested_left_mps == 0.0f && planner_result.requested_right_mps == 0.0f)
        {
            WheelSpeedControlLoop_Reset(&speed_loop);
            ChassisService_ResetPidTargets();
            for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
            {
                MotorOutputCoordinator_StopMotor(&chassis_state, (motor_id_t)i);
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
                MotorOutputCoordinator_StopMotor(&chassis_state, (motor_id_t)i);
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
            MotorOutputCoordinator_SetMotorWithPower(&chassis_state,
                                                     (motor_id_t)i,
                                                     permille,
                                                     &input->power.value,
                                                     &param_sync.params);
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

void MotionControl_StepWithInput(const motion_control_input_t *input, motion_control_event_t *event)
{
    motion_control_event_t ignored_event = {0};
    uint32_t               primask       = PlatformCritical_Enter();

    if (event == 0)
    {
        event = &ignored_event;
    }
    *event              = (motion_control_event_t){0};
    control_step_active = 1U;
    PlatformCritical_Exit(primask);
    if (input != 0 && input->wheel.validity != 0UL)
    {
        for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
        {
            motion_wheel_speed_cache[index] = input->wheel.value.speed_mps[index];
        }
    }
    if (input == 0 || input->nominal_period_ms == 0UL)
    {
        MotionControl_SetEvent(event, MOTION_EVENT_SAFETY_PERMIT_STALE, (input != 0) ? input->now_ms : 0UL);
        MotionControl_EmergencyStop();
    }
    else
    {
        ChassisService_StepImpl(input, event);
    }
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
