#include "safety_management_service.h"

#include "battery_guard.h"
#include "chassis_layout.h"
#include "command_management_service.h"
#include "motor_driver.h"
#include "platform_critical.h"
#include "platform_time.h"
#include "power_management_service.h"
#include "safety_fault_policy.h"
#include "state_estimation_service.h"
#include "system_monitoring_service.h"

static safety_management_config_t safety_config;
static safety_management_status_t monitor_state;
static uint8_t                    overcurrent_count[MOTOR_ID_COUNT];
static uint32_t                   current_observe_over_limit_count[MOTOR_ID_COUNT];
static uint32_t                   current_fault_would_latch_count[MOTOR_ID_COUNT];
static uint8_t                    motor_output_active[MOTOR_ID_COUNT];
static uint8_t                    startup_blank_armed[MOTOR_ID_COUNT];
static uint32_t                   overcurrent_blank_until_ms[MOTOR_ID_COUNT];
static uint32_t                   inactive_since_ms[MOTOR_ID_COUNT];
static battery_guard_t            battery_guard;
static uint8_t                    emergency_stop;
static uint8_t                    fault_stop;
static uint8_t                    maintenance_lock;
static uint8_t                    safety_initialized;
static uint32_t                   gate_decision_generation;

static const uint32_t overcurrent_flags[MOTOR_ID_COUNT] = {
    SYSTEM_ERROR_M1_OVERCURRENT,
    SYSTEM_ERROR_M2_OVERCURRENT,
    SYSTEM_ERROR_M3_OVERCURRENT,
    SYSTEM_ERROR_M4_OVERCURRENT,
};

static uint8_t SafetyManagement_CurrentBelowFaultThreshold(uint8_t motor, float current_a)
{
    return (current_a <= safety_config.current_fault_a[motor]) ? 1U : 0U;
}

static uint8_t SafetyManagement_BatterySampleValid(const power_management_status_t *adc_state)
{
    const uint32_t invalid_mask =
        ADC_MONITOR_INVALID_NOT_READY | ADC_MONITOR_INVALID_NO_NEW_SAMPLE | ADC_MONITOR_INVALID_DMA_ERROR;

    return (adc_state != 0 && adc_state->samples_ready != 0U && adc_state->raw_sample_count > 0U
            && (adc_state->invalid_reason_flags & invalid_mask) == 0UL)
               ? 1U
               : 0U;
}

static uint8_t SafetyManagement_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (((int32_t)(now_ms - deadline_ms)) >= 0) ? 1U : 0U;
}

static void SafetyManagement_UpdateOvercurrentCounters(const power_management_status_t *adc_state,
                                                       const uint8_t                    blanked[MOTOR_ID_COUNT],
                                                       const uint8_t                    previous_count[MOTOR_ID_COUNT],
                                                       uint8_t                          next_count[MOTOR_ID_COUNT],
                                                       uint32_t                        *new_latched_flags)
{
    uint8_t debounce_count;

    debounce_count = (uint8_t)((safety_config.current_fault_debounce_ms + safety_config.update_period_ms - 1U)
                               / safety_config.update_period_ms);
    if (debounce_count == 0U)
    {
        debounce_count = 1U;
    }
    if (safety_config.overcurrent_fault_enabled == 0U)
    {
        for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
        {
            next_count[i] = 0U;
        }
        return;
    }

    if (adc_state->current_control_valid == 0U)
    {
        for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
        {
            next_count[i] = 0U;
        }
        return;
    }

    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        next_count[i] = previous_count[i];
        if (ChassisLayout_MotorEnabled((motor_id_t)i) == 0U)
        {
            next_count[i] = 0U;
            continue;
        }
        if (blanked[i] != 0U)
        {
            next_count[i] = 0U;
            continue;
        }
        if (adc_state->current_a[i] > safety_config.current_fault_a[i])
        {
            if (next_count[i] < debounce_count)
            {
                next_count[i]++;
            }
            if (next_count[i] >= debounce_count && new_latched_flags != 0)
            {
                *new_latched_flags |= overcurrent_flags[i];
            }
        }
        else
        {
            next_count[i] = 0U;
        }
    }
}

static void SafetyManagement_UpdateCurrentDryRun(const power_management_status_t *adc_state,
                                                 const uint8_t                    blanked[MOTOR_ID_COUNT])
{
    uint8_t debounce_count;
    (void)blanked;

    debounce_count = (uint8_t)((safety_config.current_fault_debounce_ms + safety_config.update_period_ms - 1U)
                               / safety_config.update_period_ms);
    if (debounce_count == 0U)
    {
        debounce_count = 1U;
    }

    if (adc_state == 0 || adc_state->current_control_valid == 0U)
    {
        return;
    }
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        uint8_t mask = (uint8_t)(1U << i);
        if (ChassisLayout_MotorEnabled((motor_id_t)i) == 0U || (adc_state->current_control_valid_mask & mask) == 0U
            || motor_output_active[i] == 0U)
        {
            continue;
        }
        if (adc_state->current_a[i] > safety_config.current_observe_a[i])
        {
            current_observe_over_limit_count[i]++;
            if (safety_config.overcurrent_fault_enabled == 0U || overcurrent_count[i] + 1U >= debounce_count)
            {
                current_fault_would_latch_count[i]++;
            }
        }
    }
}

static uint8_t SafetyManagement_ConfigValid(const safety_management_config_t *config)
{
    return (config != 0 && config->battery_low_warn_v > config->battery_critical_v
            && config->battery_low_clear_v > config->battery_low_warn_v
            && config->battery_recover_v > config->battery_critical_v && config->battery_critical_debounce_ms > 0UL
            && config->battery_recover_debounce_ms > 0UL && config->update_period_ms > 0UL
            && config->overcurrent_startup_blank_ms > 0UL && config->overcurrent_startup_rearm_ms > 0UL
            && config->current_fault_debounce_ms > 0U && config->battery_low_monitor_enabled <= 1U
            && config->overcurrent_fault_enabled <= 1U)
               ? 1U
               : 0U;
}

static uint8_t SafetyManagement_MotionAllowed(void)
{
    return (emergency_stop == 0U && fault_stop == 0U && maintenance_lock == 0U) ? 1U : 0U;
}

uint8_t SafetyManagement_Init(const safety_management_config_t *config)
{
    platform_critical_state_t critical;

    if (SafetyManagement_ConfigValid(config) == 0U)
    {
        return 0U;
    }
    critical         = PlatformCritical_Enter();
    safety_config    = *config;
    monitor_state    = (safety_management_status_t){0};
    emergency_stop   = 0U;
    fault_stop       = 0U;
    maintenance_lock = 0U;
    gate_decision_generation++;
    if (gate_decision_generation == 0UL)
    {
        gate_decision_generation = 1UL;
    }
    monitor_state.motion_allowed = 1U;
    monitor_state.generation     = 1UL;
    safety_initialized           = 1U;
    PlatformCritical_Exit(critical);
    BatteryGuard_Init(&battery_guard);
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        overcurrent_count[i]                = 0U;
        current_observe_over_limit_count[i] = 0UL;
        current_fault_would_latch_count[i]  = 0UL;
        motor_output_active[i]              = 0U;
        startup_blank_armed[i]              = 1U;
        overcurrent_blank_until_ms[i]       = 0U;
        inactive_since_ms[i]                = 0U;
    }
    CommandManagement_SetMotionGate(1U, gate_decision_generation);
    return 1U;
}

void SafetyManagement_SetCurrentThresholds(const float observe_a[SAFETY_MANAGEMENT_MOTOR_COUNT],
                                           const float fault_a[SAFETY_MANAGEMENT_MOTOR_COUNT],
                                           uint16_t    debounce_ms)
{
    platform_critical_state_t critical;

    if (observe_a == 0 || fault_a == 0 || debounce_ms == 0U)
    {
        return;
    }
    critical = PlatformCritical_Enter();
    for (uint8_t index = 0U; index < SAFETY_MANAGEMENT_MOTOR_COUNT; ++index)
    {
        safety_config.current_observe_a[index] = observe_a[index];
        safety_config.current_fault_a[index]   = fault_a[index];
    }
    safety_config.current_fault_debounce_ms = debounce_ms;
    PlatformCritical_Exit(critical);
}

uint8_t SafetyManagement_IsInitialized(void)
{
    return safety_initialized;
}

void SafetyManagement_Update(void)
{
    power_management_status_t       adc_state;
    state_estimation_wheel_status_t encoder_state;
    motor_driver_state_t            motor_state;
    uint8_t                         active_source;
    system_monitoring_task_health_t task_health;
    safety_management_status_t      next_state;
    uint8_t                         previous_overcurrent_count[MOTOR_ID_COUNT];
    uint8_t                         next_overcurrent_count[MOTOR_ID_COUNT];
    uint8_t                         overcurrent_blanked[MOTOR_ID_COUNT];
    uint32_t                        new_latched_flags = 0U;
    uint32_t                        latched_after_commit;
    uint8_t                         request_fault_stop = 0U;
    uint8_t                         release_fault_stop = 0U;
    uint8_t                         battery_sample_valid;
    uint8_t                         battery_critical_latched;
    battery_guard_result_t          battery_result;
    uint32_t                        auto_clear_latched_flags = 0UL;
    uint32_t                        now_ms;
    uint32_t                        primask;

    now_ms = PlatformTime_TaskNowMs();
    SystemMonitoring_UpdateTimeouts(now_ms);
    PowerManagement_Update();
    MotorDriver_UpdateFaults();
    (void)PowerManagement_GetStatus(&adc_state);
    (void)StateEstimation_GetWheel(&encoder_state);
    MotorDriver_GetState(&motor_state);
    active_source = (uint8_t)CommandManagement_GetActiveSource(now_ms);
    (void)SystemMonitoring_GetTaskHealth(&task_health);

    primask                  = PlatformCritical_Enter();
    battery_critical_latched = ((monitor_state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) != 0U) ? 1U : 0U;
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        previous_overcurrent_count[i] = overcurrent_count[i];
    }
    PlatformCritical_Exit(primask);

    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        uint8_t active =
            (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U && motor_state.effective_pwm[i] != 0) ? 1U : 0U;
        if (active != 0U)
        {
            inactive_since_ms[i] = now_ms;
            if (motor_output_active[i] == 0U && startup_blank_armed[i] != 0U)
            {
                overcurrent_blank_until_ms[i] = now_ms + safety_config.overcurrent_startup_blank_ms;
                startup_blank_armed[i]        = 0U;
            }
            motor_output_active[i] = 1U;
        }
        else
        {
            if (motor_output_active[i] != 0U)
            {
                inactive_since_ms[i] = now_ms;
            }
            motor_output_active[i] = 0U;
            if (SafetyManagement_TimeReached(now_ms, inactive_since_ms[i] + safety_config.overcurrent_startup_rearm_ms)
                != 0U)
            {
                startup_blank_armed[i]        = 1U;
                overcurrent_blank_until_ms[i] = 0U;
            }
        }
        overcurrent_blanked[i] =
            (startup_blank_armed[i] == 0U && SafetyManagement_TimeReached(now_ms, overcurrent_blank_until_ms[i]) == 0U)
                ? 1U
                : 0U;
    }

    SafetyManagement_UpdateCurrentDryRun(&adc_state, overcurrent_blanked);

    next_state                            = (safety_management_status_t){0};
    next_state.battery_voltage            = adc_state.battery_voltage;
    next_state.left_current_a             = adc_state.left_current_a;
    next_state.right_current_a            = adc_state.right_current_a;
    next_state.current_control_valid      = adc_state.current_control_valid;
    next_state.current_control_valid_mask = adc_state.current_control_valid_mask;
    next_state.control_mode               = active_source;
    next_state.task_timeout_mask          = SystemMonitoring_GetTimeoutMask();
    for (uint8_t i = 0U; i < (uint8_t)SYSTEM_MONITORING_TASK_COUNT; ++i)
    {
        next_state.task_last_heartbeat_ms[i] = task_health.last_heartbeat_ms[i];
        next_state.task_timeout_count[i]     = task_health.timeout_count[i];
        next_state.task_timed_out[i]         = task_health.timed_out[i];
    }
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        next_state.motor_current_a[i]                  = adc_state.current_a[i];
        next_state.current_observe_over_limit_count[i] = current_observe_over_limit_count[i];
        next_state.current_fault_would_latch_count[i]  = current_fault_would_latch_count[i];
        if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U && motor_state.fault_active[i] != 0U)
        {
            next_state.motor_fault_mask |= (uint8_t)(1U << i);
        }
    }

    SafetyManagement_UpdateOvercurrentCounters(&adc_state,
                                               overcurrent_blanked,
                                               previous_overcurrent_count,
                                               next_overcurrent_count,
                                               &new_latched_flags);
    battery_sample_valid = SafetyManagement_BatterySampleValid(&adc_state);
    BatteryGuard_Update(&battery_guard,
                        &safety_config,
                        next_state.battery_voltage,
                        battery_sample_valid,
                        battery_critical_latched,
                        now_ms,
                        &battery_result);
    if (battery_result.warning_active != 0U)
    {
        next_state.error_flags |= SYSTEM_ERROR_LOW_BATTERY;
    }
    if (battery_result.latch_critical != 0U)
    {
        new_latched_flags |= SYSTEM_ERROR_BATTERY_CRITICAL;
    }
    if (battery_result.clear_critical != 0U)
    {
        auto_clear_latched_flags |= SYSTEM_ERROR_BATTERY_CRITICAL;
    }
    if (encoder_state.speed_valid_all == 0U)
    {
        next_state.error_flags |= SYSTEM_ERROR_ENCODER_INVALID;
    }
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U && motor_state.fault_active[i] != 0U)
        {
            new_latched_flags |= SYSTEM_ERROR_DRV_FAULT;
        }
    }
    if (motor_state.tim1_break_latched != 0U)
    {
        new_latched_flags |= SYSTEM_ERROR_TIM_BREAK;
    }

    primask = PlatformCritical_Enter();
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        overcurrent_count[i] = next_overcurrent_count[i];
    }
    next_state.latched_error_flags = monitor_state.latched_error_flags | new_latched_flags;
    next_state.latched_error_flags &= ~auto_clear_latched_flags;
    latched_after_commit = next_state.latched_error_flags;
    if (emergency_stop != 0U)
    {
        next_state.error_flags |= SYSTEM_ERROR_ESTOP;
    }
    if (fault_stop != 0U)
    {
        next_state.error_flags |= SYSTEM_ERROR_FAULT_STOP;
    }
    next_state.error_flags |= latched_after_commit;
    if (SafetyFaultPolicy_RequiresFaultStop(latched_after_commit) != 0U)
    {
        request_fault_stop = 1U;
        next_state.error_flags |= SYSTEM_ERROR_FAULT_STOP;
    }
    else if (auto_clear_latched_flags != 0UL)
    {
        release_fault_stop = 1U;
    }
    next_state.emergency_stop   = emergency_stop;
    next_state.fault_stop       = fault_stop;
    next_state.maintenance_lock = maintenance_lock;
    next_state.motion_allowed   = SafetyManagement_MotionAllowed();
    next_state.generation       = monitor_state.generation + 1UL;
    monitor_state               = next_state;
    PlatformCritical_Exit(primask);

    if (request_fault_stop != 0U)
    {
        SafetyManagement_SetFaultStop(1U);
    }
    else if (release_fault_stop != 0U)
    {
        SafetyManagement_SetFaultStop(0U);
    }
}

uint32_t SafetyManagement_GetStatus(safety_management_status_t *state)
{
    uint32_t generation;
    uint32_t primask;

    if (state == 0)
    {
        return 0UL;
    }

    primask    = PlatformCritical_Enter();
    *state     = monitor_state;
    generation = monitor_state.generation;
    PlatformCritical_Exit(primask);
    return generation;
}

void SafetyManagement_ClearLatchedFaults(uint32_t mask)
{
    uint32_t                        clearable = SafetyFaultPolicy_ManualClearMask(mask);
    safety_management_status_t      snapshot;
    motor_driver_state_t            motor_state;
    state_estimation_wheel_status_t encoder_state;
    uint8_t                         clear_fault_stop = 0U;
    uint32_t                        latched_after_clear;
    safety_management_status_t      next;
    uint32_t                        primask;

    MotorDriver_GetState(&motor_state);
    (void)StateEstimation_GetWheel(&encoder_state);

    primask  = PlatformCritical_Enter();
    snapshot = monitor_state;
    PlatformCritical_Exit(primask);

    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U && (mask & overcurrent_flags[i]) != 0U
            && (((snapshot.current_control_valid_mask & (uint8_t)(1U << i)) == 0U)
                || SafetyManagement_CurrentBelowFaultThreshold(i, snapshot.motor_current_a[i]) == 0U))
        {
            clearable &= ~overcurrent_flags[i];
        }
    }
    if ((mask & SYSTEM_ERROR_DRV_FAULT) != 0U)
    {
        for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
        {
            if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U && motor_state.fault_active[i] != 0U)
            {
                clearable &= ~SYSTEM_ERROR_DRV_FAULT;
            }
        }
    }
    if ((mask & SYSTEM_ERROR_TIM_BREAK) != 0U && MotorDriver_ClearBreakLatch() == 0U)
    {
        clearable &= ~SYSTEM_ERROR_TIM_BREAK;
    }
    if ((mask & SYSTEM_ERROR_ENCODER_FEEDBACK_LOST) != 0U)
    {
        for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
        {
            if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U
                && (encoder_state.speed_valid[i] == 0U || motor_state.requested_pwm[i] != 0
                    || motor_state.applied_pwm[i] != 0 || motor_state.effective_pwm[i] != 0))
            {
                clearable &= ~SYSTEM_ERROR_ENCODER_FEEDBACK_LOST;
            }
        }
    }

    primask = PlatformCritical_Enter();
    next    = monitor_state;
    next.latched_error_flags &= ~clearable;
    latched_after_clear = next.latched_error_flags;
    if (SafetyFaultPolicy_RequiresFaultStop(latched_after_clear) == 0U)
    {
        for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
        {
            overcurrent_count[i] = 0U;
        }
        clear_fault_stop = 1U;
    }
    next.error_flags &= ~clearable;
    next.generation++;
    monitor_state = next;
    PlatformCritical_Exit(primask);

    if (clear_fault_stop != 0U)
    {
        SafetyManagement_SetFaultStop(0U);
    }
}

void SafetyManagement_LatchEncoderFeedbackFault(void)
{
    safety_management_status_t next;
    uint32_t                   primask = PlatformCritical_Enter();

    next = monitor_state;
    next.latched_error_flags |= SYSTEM_ERROR_ENCODER_FEEDBACK_LOST;
    next.error_flags |= SYSTEM_ERROR_ENCODER_FEEDBACK_LOST | SYSTEM_ERROR_FAULT_STOP;
    next.generation++;
    monitor_state = next;
    PlatformCritical_Exit(primask);
    SafetyManagement_SetFaultStop(1U);
}

uint8_t SafetyManagement_HasLatchedFault(void)
{
    uint8_t  has_fault;
    uint32_t primask = PlatformCritical_Enter();
    has_fault        = (monitor_state.latched_error_flags != 0U) ? 1U : 0U;
    PlatformCritical_Exit(primask);
    return has_fault;
}

void SafetyManagement_SetEmergencyStop(uint8_t enabled)
{
    safety_management_status_t next;
    uint8_t                    decision;
    uint8_t                    value = (enabled != 0U) ? 1U : 0U;
    uint32_t                   decision_generation;
    uint32_t                   primask = PlatformCritical_Enter();

    if (emergency_stop == value)
    {
        PlatformCritical_Exit(primask);
        return;
    }
    emergency_stop = value;
    gate_decision_generation++;
    next                = monitor_state;
    next.emergency_stop = value;
    if (value != 0U)
    {
        next.error_flags |= SYSTEM_ERROR_ESTOP;
    }
    else
    {
        next.error_flags &= ~SYSTEM_ERROR_ESTOP;
    }
    decision            = SafetyManagement_MotionAllowed();
    next.motion_allowed = decision;
    next.generation++;
    monitor_state       = next;
    decision_generation = gate_decision_generation;
    PlatformCritical_Exit(primask);
    CommandManagement_SetMotionGate(decision, decision_generation);
}

void SafetyManagement_SetFaultStop(uint8_t enabled)
{
    safety_management_status_t next;
    uint8_t                    decision;
    uint8_t                    value = (enabled != 0U) ? 1U : 0U;
    uint32_t                   decision_generation;
    uint32_t                   primask = PlatformCritical_Enter();

    if (fault_stop == value)
    {
        PlatformCritical_Exit(primask);
        return;
    }
    fault_stop = value;
    gate_decision_generation++;
    next            = monitor_state;
    next.fault_stop = value;
    if (value != 0U)
    {
        next.error_flags |= SYSTEM_ERROR_FAULT_STOP;
    }
    else
    {
        next.error_flags &= ~SYSTEM_ERROR_FAULT_STOP;
    }
    decision            = SafetyManagement_MotionAllowed();
    next.motion_allowed = decision;
    next.generation++;
    monitor_state       = next;
    decision_generation = gate_decision_generation;
    PlatformCritical_Exit(primask);
    CommandManagement_SetMotionGate(decision, decision_generation);
}

uint8_t SafetyManagement_BeginMaintenance(void)
{
    safety_management_status_t next;
    uint32_t                   decision_generation;
    uint32_t                   primask = PlatformCritical_Enter();

    if (maintenance_lock != 0U)
    {
        PlatformCritical_Exit(primask);
        return 0U;
    }
    maintenance_lock = 1U;
    gate_decision_generation++;
    next                  = monitor_state;
    next.maintenance_lock = 1U;
    next.motion_allowed   = 0U;
    next.generation++;
    monitor_state       = next;
    decision_generation = gate_decision_generation;
    PlatformCritical_Exit(primask);
    CommandManagement_SetMotionGate(0U, decision_generation);
    return 1U;
}

void SafetyManagement_EndMaintenance(void)
{
    safety_management_status_t next;
    uint8_t                    decision;
    uint32_t                   decision_generation;
    uint32_t                   primask = PlatformCritical_Enter();

    if (maintenance_lock == 0U)
    {
        PlatformCritical_Exit(primask);
        return;
    }
    maintenance_lock = 0U;
    gate_decision_generation++;
    decision              = SafetyManagement_MotionAllowed();
    next                  = monitor_state;
    next.maintenance_lock = 0U;
    next.motion_allowed   = decision;
    next.generation++;
    monitor_state       = next;
    decision_generation = gate_decision_generation;
    PlatformCritical_Exit(primask);
    CommandManagement_SetMotionGate(decision, decision_generation);
}

uint8_t SafetyManagement_IsEmergencyStop(void)
{
    uint8_t  value;
    uint32_t primask = PlatformCritical_Enter();

    value = emergency_stop;
    PlatformCritical_Exit(primask);
    return value;
}

uint8_t SafetyManagement_IsFaultStop(void)
{
    uint8_t  value;
    uint32_t primask = PlatformCritical_Enter();

    value = fault_stop;
    PlatformCritical_Exit(primask);
    return value;
}

uint8_t SafetyManagement_IsMaintenanceLocked(void)
{
    uint8_t  value;
    uint32_t primask = PlatformCritical_Enter();

    value = maintenance_lock;
    PlatformCritical_Exit(primask);
    return value;
}

uint8_t SafetyManagement_IsMotionAllowed(void)
{
    uint8_t  value;
    uint32_t primask = PlatformCritical_Enter();

    value = SafetyManagement_MotionAllowed();
    PlatformCritical_Exit(primask);
    return value;
}
