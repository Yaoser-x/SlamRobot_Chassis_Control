#include "chassis_runtime_coordinator.h"

#include "command_management_service.h"
#include "control_mode_coordinator.h"
#include "line_following_service.h"
#include "motor_driver_snapshot_adapter.h"
#include "motion_control_service.h"
#include "parameter_management_service.h"
#include "power_management_service.h"
#include "power_on_self_test_service.h"
#include "safety_management_service.h"
#include "safety_workflow_coordinator.h"
#include "state_estimation_service.h"
#include "platform_critical.h"
#include "watchdog_completion_gate.h"
#include "robot_config.h"
#include "system_monitoring_service.h"

#define CHASSIS_RUNTIME_IMU_FRESH_MS                 100U
#define CHASSIS_RUNTIME_MOTOR_COMPLETION_TIMEOUT_MS  20U
#define CHASSIS_RUNTIME_SAFETY_COMPLETION_TIMEOUT_MS 40U
#define CHASSIS_RUNTIME_IWDG_NOMINAL_TIMEOUT_MS      800U
static task_completion_t     motor_completion;
static task_completion_t     safety_completion;
static task_completion_t     pending_watchdog_motor;
static task_completion_t     pending_watchdog_safety;
static watchdog_gate_state_t watchdog_gate_state;
static uint8_t               watchdog_feed_pending;
static uint8_t               communication_initialized;
static uint8_t               boot_qualified;

static const watchdog_gate_config_t watchdog_gate_config = {
    .motor_timeout_ms    = CHASSIS_RUNTIME_MOTOR_COMPLETION_TIMEOUT_MS,
    .safety_timeout_ms   = CHASSIS_RUNTIME_SAFETY_COMPLETION_TIMEOUT_MS,
    .hardware_timeout_ms = CHASSIS_RUNTIME_IWDG_NOMINAL_TIMEOUT_MS,
};

static void ChassisRuntimeCoordinator_CommitCompletion(task_completion_t *completion, uint32_t now_ms)
{
    uint32_t primask = PlatformCritical_Enter();

    completion->generation++;
    if (completion->generation == 0UL)
    {
        completion->generation = 1UL;
    }
    completion->completed_at_ms = now_ms;
    PlatformCritical_Exit(primask);
}

static task_completion_t ChassisRuntimeCoordinator_CopyCompletion(const task_completion_t *completion)
{
    task_completion_t snapshot;
    uint32_t          primask = PlatformCritical_Enter();

    snapshot = *completion;
    PlatformCritical_Exit(primask);
    return snapshot;
}

static uint8_t ChassisRuntimeCoordinator_OutputsZero(const motion_control_status_t *motion)
{
    for (uint8_t index = 0U; index < MOTION_CONTROL_MOTOR_COUNT; ++index)
    {
        if (motion->motor_effective_output_permille[index] != 0 || motion->motor_output_permille[index] != 0)
        {
            return 0U;
        }
    }
    return 1U;
}

static void ChassisRuntimeCoordinator_CollectCapabilities(uint32_t now_ms, safety_capability_permit_t *capabilities)
{
    parameter_management_status_t params;
    power_management_status_t     power;
    state_estimation_status_t     state;
    motion_control_status_t       motion;
    safety_management_status_t    safety;
    power_on_self_test_result_t   post;
    line_following_status_t       line;
    task_completion_t             motor_snapshot;
    safety_capability_input_t     input = {0};

    (void)ParameterManagement_GetStatus(&params);
    (void)PowerManagement_GetStatus(&power);
    (void)StateEstimation_GetStatus(now_ms, &state);
    (void)MotionControl_GetStatus(&motion);
    (void)SafetyManagement_GetStatus(&safety);
    (void)LineFollowing_GetStatus(&line);
    PowerOnSelfTest_GetResult(&post);
    motor_snapshot        = ChassisRuntimeCoordinator_CopyCompletion(&motor_completion);
    input.parameter_ready = (params.initialized != 0U && params.generation != 0UL && communication_initialized != 0U
                             && motor_snapshot.generation != 0UL && post.done != 0U)
                                ? 1U
                                : 0U;
    input.current_control_valid = (power.current_zero_valid != 0U && power.current_control_valid != 0U) ? 1U : 0U;
    input.encoder_valid         = state.wheel.speed_valid_all;
    input.motor_driver_ok       = (safety.motor_fault_mask == 0U) ? 1U : 0U;
    input.tim_break_clear       = ((safety.error_flags & SYSTEM_ERROR_TIM_BREAK) == 0UL) ? 1U : 0U;
    input.safety_fault_free =
        (safety.emergency_stop == 0U && safety.fault_stop == 0U && safety.latched_error_flags == 0UL) ? 1U : 0U;
    input.line_sensor_valid = line.sensor_valid;
    input.imu_online        = (state.imu.online != 0U && state.imu.chip_id != 0U) ? 1U : 0U;
    input.imu_fresh =
        (state.imu_fresh != 0U && (uint32_t)(now_ms - state.imu.last_update_ms) <= CHASSIS_RUNTIME_IMU_FRESH_MS) ? 1U
                                                                                                                 : 0U;
    input.imu_calibrated    = state.imu.gyro_calibrated;
    input.imu_quality_flags = state.imu.quality_flags;
    input.fatal_post_flags  = post.fatal_error_flags;
    SafetyManagement_EvaluateCapabilities(&input, capabilities);

    if (boot_qualified == 0U && capabilities->base_motion != 0U && ChassisRuntimeCoordinator_OutputsZero(&motion) == 0U)
    {
        capabilities->base_motion     = 0U;
        capabilities->manual          = 0U;
        capabilities->remote_velocity = 0U;
        capabilities->line            = 0U;
    }
}

static void ChassisRuntimeCoordinator_CollectSafetyInput(uint32_t now_ms, safety_management_input_t *input)
{
    app_motor_driver_snapshot_t motor;

    *input        = (safety_management_input_t){0};
    input->now_ms = now_ms;

    input->power.generation     = PowerManagement_GetStatus(&input->power.value);
    input->power.sample_time_ms = now_ms;
    input->power.validity       = (input->power.generation != 0UL) ? 1UL : 0UL;
    input->power.quality        = input->power.value.invalid_reason_flags;

    input->wheel.generation     = StateEstimation_GetWheel(&input->wheel.value);
    input->wheel.sample_time_ms = input->wheel.value.last_update_ms;
    input->wheel.validity       = (input->wheel.generation != 0UL) ? 1UL : 0UL;
    input->wheel.quality        = input->wheel.value.current_anomaly_mask;

    input->motor.generation               = AppMotorDriverAdapter_GetSnapshot(&motor);
    input->motor.sample_time_ms           = now_ms;
    input->motor.validity                 = (input->motor.generation != 0UL) ? 1UL : 0UL;
    input->motor.quality                  = motor.tim1_break_latched;
    input->motor.value.tim1_break_latched = motor.tim1_break_latched;
    input->motor.value.enabled_mask       = motor.enabled_mask;
    for (uint8_t index = 0U; index < APP_MOTOR_DRIVER_COUNT; ++index)
    {
        input->motor.value.requested_pwm[index] = motor.requested_pwm[index];
        input->motor.value.applied_pwm[index]   = motor.applied_pwm[index];
        input->motor.value.effective_pwm[index] = motor.effective_pwm[index];
        input->motor.value.fault_active[index]  = motor.fault_active[index];
        if (motor.fault_active[index] != 0U)
        {
            input->motor.quality |= (uint32_t)(1UL << (index + 1U));
        }
    }

    input->command.generation     = CommandManagement_GetStatus(now_ms, &input->command.value);
    input->command.sample_time_ms = now_ms;
    input->command.validity       = (input->command.generation != 0UL) ? 1UL : 0UL;

    input->system.generation     = SystemMonitoring_GetStatus(&input->system.value);
    input->system.sample_time_ms = now_ms;
    input->system.validity       = (input->system.generation != 0UL) ? 1UL : 0UL;
}

static uint8_t ChassisRuntimeCoordinator_SourcePermitted(command_source_t                  source,
                                                         const safety_capability_permit_t *capabilities)
{
    switch (source)
    {
        case COMMAND_SOURCE_HOST:
        case COMMAND_SOURCE_ESP12F:
            return capabilities->remote_velocity;
        case COMMAND_SOURCE_PS2:
            return capabilities->manual;
        case COMMAND_SOURCE_LINE:
            return capabilities->line;
        case COMMAND_SOURCE_DEBUG:
        case COMMAND_SOURCE_NONE:
        default:
            return capabilities->base_motion;
    }
}

static uint8_t ChassisRuntimeCoordinator_CapabilitySourceMask(const safety_capability_permit_t *capabilities)
{
    uint8_t mask = 0U;

    if (capabilities->remote_velocity != 0U)
    {
        mask |= COMMAND_SOURCE_MASK_REMOTE;
    }
    if (capabilities->manual != 0U)
    {
        mask |= COMMAND_SOURCE_MASK_PS2;
    }
    if (capabilities->line != 0U)
    {
        mask |= COMMAND_SOURCE_MASK_LINE;
    }
    if (capabilities->maintenance_motion != 0U)
    {
        mask |= COMMAND_SOURCE_MASK_DEBUG;
    }
    return mask;
}

static uint8_t ChassisRuntimeCoordinator_DiagnosticFactsValid(void)
{
    power_management_status_t  power;
    safety_management_status_t safety;

    (void)PowerManagement_GetStatus(&power);
    (void)SafetyManagement_GetStatus(&safety);
    return (power.current_zero_valid != 0U && power.current_control_valid != 0U
            && (safety.error_flags & (SYSTEM_ERROR_DRV_FAULT | SYSTEM_ERROR_TIM_BREAK)) == 0UL
            && safety.emergency_stop == 0U && safety.fault_stop == 0U)
               ? 1U
               : 0U;
}

static void ChassisRuntimeCoordinator_CollectMotionInput(uint32_t now_ms, motion_control_input_t *input)
{
    command_management_status_t command_status;
    safety_management_status_t  safety_status;
    state_estimation_status_t   state_status;

    *input                       = (motion_control_input_t){0};
    input->now_ms                = now_ms;
    input->nominal_period_ms     = RobotConfig_GetDefault()->tasks[APP_TASK_MOTOR].period_ms;
    input->parameters.generation = ParameterManagement_GetSnapshot(&input->parameters.value);
    input->parameters.validity   = (input->parameters.generation != 0UL) ? 1UL : 0UL;

    (void)StateEstimation_GetStatus(now_ms, &state_status);
    input->wheel.value          = state_status.wheel;
    input->wheel.sample_time_ms = state_status.wheel.last_update_ms;
    input->wheel.generation     = state_status.wheel_generation;
    input->wheel.validity       = (state_status.wheel_generation != 0UL) ? 1UL : 0UL;
    input->wheel.quality =
        (uint32_t)state_status.wheel.current_anomaly_mask | ((uint32_t)state_status.wheel.latched_for_host_mask << 8U);
    input->imu.value          = state_status.imu;
    input->imu.sample_time_ms = state_status.imu.last_update_ms;
    input->imu.generation     = state_status.imu_sample_generation;
    input->imu.validity       = (state_status.imu_sample_generation != 0UL) ? 1UL : 0UL;
    input->imu.quality        = state_status.imu.quality_flags;

    input->power.generation = PowerManagement_GetStatus(&input->power.value);
    input->power.validity   = (input->power.generation != 0UL) ? 1UL : 0UL;
    input->power.quality    = input->power.value.invalid_reason_flags;

    input->command.validity       = CommandManagement_GetActiveSnapshot(now_ms, &input->command.value, &command_status);
    input->command.sample_time_ms = input->command.value.timestamp_ms;
    input->command.generation     = command_status.generation;
    input->command.revoke_generation = command_status.motion_revoke_generation;

    (void)SafetyManagement_GetStatus(&safety_status);
    input->safety_permit.value          = safety_status.motion_permit;
    input->safety_permit.sample_time_ms = safety_status.motion_permit.issued_at_ms;
    input->safety_permit.generation     = safety_status.motion_permit.generation;
    input->safety_permit.validity       = (safety_status.motion_permit.generation != 0UL) ? 1UL : 0UL;
    input->normal_motion_allowed        = safety_status.motion_allowed;
    input->diagnostic_motion_allowed    = SafetyManagement_IsDiagnosticMotionAllowed();
}

static void ChassisRuntimeCoordinator_RouteMotionEvent(const motion_control_event_t *event)
{
    if ((event->flags & (MOTION_EVENT_DRIVER_FAULT | MOTION_EVENT_SAFETY_PERMIT_STALE)) != 0UL)
    {
        AppSafetyWorkflow_SetFaultStop(1U);
    }
    if ((event->flags & MOTION_EVENT_ENCODER_FEEDBACK_LOST) != 0UL)
    {
        AppSafetyWorkflow_LatchEncoderFeedbackFault();
    }
    if ((event->flags & MOTION_EVENT_COMMAND_REVOKE) != 0UL)
    {
        CommandManagement_ClearAll();
    }
}

void ChassisRuntimeCoordinator_Init(void)
{
    motor_completion          = (task_completion_t){0};
    safety_completion         = (task_completion_t){0};
    pending_watchdog_motor    = (task_completion_t){0};
    pending_watchdog_safety   = (task_completion_t){0};
    watchdog_gate_state       = (watchdog_gate_state_t){0};
    watchdog_feed_pending     = 0U;
    communication_initialized = 1U;
    boot_qualified            = 0U;
    SafetyManagement_ApplyRuntimePermit(0U, SAFETY_STATE_BOOT_SAFE);
    SafetyManagement_ApplyDiagnosticPermit(0U);
    AppSafetyWorkflow_SynchronizeCommandGate();
}

void ChassisRuntimeCoordinator_RunMotorCycle(uint32_t now_ms)
{
    motion_control_input_t input;
    motion_control_event_t event;

    StateEstimation_UpdateWheel(now_ms);
    PowerManagement_UpdateStationary();
    ChassisRuntimeCoordinator_CollectMotionInput(now_ms, &input);
    MotionControl_StepWithInput(&input, &event);
    ChassisRuntimeCoordinator_RouteMotionEvent(&event);
}

void ChassisRuntimeCoordinator_CommitMotorCycle(uint32_t now_ms)
{
    ChassisRuntimeCoordinator_CommitCompletion(&motor_completion, now_ms);
}

void ChassisRuntimeCoordinator_RunSafetyCycle(uint32_t now_ms)
{
    param_model_t              params;
    uint8_t                    permit;
    command_source_t           source;
    safety_runtime_state_t     runtime_state;
    safety_capability_permit_t capabilities;
    safety_management_input_t  safety_input;

    (void)ParameterManagement_GetSnapshot(&params);
    SafetyManagement_SetCurrentThresholds(params.current_observe_a,
                                          params.current_fault_a,
                                          params.current_fault_debounce_ms);
    SystemMonitoring_UpdateTimeouts(now_ms);
    PowerManagement_Update();
    AppMotorDriverAdapter_UpdateFaults();
    ChassisRuntimeCoordinator_CollectSafetyInput(now_ms, &safety_input);
    SafetyManagement_UpdateWithInput(&safety_input);
    PostService_UpdateRuntime(now_ms);
    SafetyManagement_ApplyDiagnosticPermit(ChassisRuntimeCoordinator_DiagnosticFactsValid());
    ChassisRuntimeCoordinator_CollectCapabilities(now_ms, &capabilities);
    if (capabilities.base_motion != 0U)
    {
        boot_qualified = 1U;
    }
    (void)ControlModeCoordinator_ApplyCapabilityMask(ChassisRuntimeCoordinator_CapabilitySourceMask(&capabilities));
    source        = CommandManagement_GetActiveSource(now_ms);
    permit        = ChassisRuntimeCoordinator_SourcePermitted(source, &capabilities);
    runtime_state = (permit == 0U) ? ((boot_qualified != 0U) ? SAFETY_STATE_STANDBY : SAFETY_STATE_BOOT_SAFE)
                                   : ((source == COMMAND_SOURCE_NONE) ? SAFETY_STATE_STANDBY : SAFETY_STATE_ACTIVE);
    SafetyManagement_ApplyCapabilityDecision(&capabilities, permit, now_ms, runtime_state);
    AppSafetyWorkflow_SynchronizeCommandGate();
}

void ChassisRuntimeCoordinator_CommitSafetyCycle(uint32_t now_ms)
{
    ChassisRuntimeCoordinator_CommitCompletion(&safety_completion, now_ms);
}

uint8_t ChassisRuntimeCoordinator_WatchdogFeedAllowed(uint32_t now_ms)
{
    task_completion_t motor_snapshot  = ChassisRuntimeCoordinator_CopyCompletion(&motor_completion);
    task_completion_t safety_snapshot = ChassisRuntimeCoordinator_CopyCompletion(&safety_completion);

    if (WatchdogCompletionGate_Evaluate(now_ms,
                                        &motor_snapshot,
                                        &safety_snapshot,
                                        &watchdog_gate_config,
                                        &watchdog_gate_state)
        != WATCHDOG_GATE_ALLOW)
    {
        watchdog_feed_pending = 0U;
        return 0U;
    }
    pending_watchdog_motor  = motor_snapshot;
    pending_watchdog_safety = safety_snapshot;
    watchdog_feed_pending   = 1U;
    return 1U;
}

void ChassisRuntimeCoordinator_CommitWatchdogFeed(void)
{
    if (watchdog_feed_pending == 0U)
    {
        return;
    }
    WatchdogCompletionGate_CommitFeed(&watchdog_gate_state, &pending_watchdog_motor, &pending_watchdog_safety);
    watchdog_feed_pending = 0U;
}

uint32_t ChassisRuntimeCoordinator_GetMotorCompletionGeneration(void)
{
    return ChassisRuntimeCoordinator_CopyCompletion(&motor_completion).generation;
}

uint32_t ChassisRuntimeCoordinator_GetSafetyCompletionGeneration(void)
{
    return ChassisRuntimeCoordinator_CopyCompletion(&safety_completion).generation;
}
