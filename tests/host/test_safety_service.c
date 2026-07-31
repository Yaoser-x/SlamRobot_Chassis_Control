#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "power_adc_driver.h"
#include "bsp_config.h"
#include "motor_hardware_layout.h"
#include "command_management_service.h"
#include "wheel_encoder_driver.h"
#include "motor_driver.h"
#include "control_config.h"
#include "parameter_management_service.h"
#include "power_management_service.h"
#include "safety_management_service.h"
#include "state_estimation_service.h"
#include "system_monitoring_service.h"

uint32_t ParameterManagement_GetSnapshot(param_model_t *params)
{
    *params = (param_model_t){0};
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        params->current_observe_a[i] = MOTOR_STALL_CURRENT_A;
        params->current_fault_a[i]   = MOTOR_STALL_CURRENT_A;
    }
    params->current_fault_debounce_ms = (uint16_t)(MOTOR_OVERCURRENT_DEBOUNCE_COUNT * CHASSIS_ADC_PERIOD_MS);
    return 1U;
}

static uint32_t                        fake_primask;
static uint32_t                        fake_tick_ms;
static power_adc_driver_state_t        fake_adc_state;
static state_estimation_wheel_status_t fake_encoder_state;
static motor_driver_state_t            fake_motor_state;
static command_source_t                fake_active_source;
static uint8_t                         fake_gate_allowed;
static uint8_t                         fake_gate_closed_count;
static uint32_t                        fake_gate_generation;
static uint8_t                         fake_break_clear_allowed;

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

uint32_t osKernelGetTickCount(void)
{
    return fake_tick_ms;
}

int32_t osDelayUntil(uint32_t ticks)
{
    fake_tick_ms = ticks;
    return 0;
}

void PowerAdcDriver_Update(void)
{
}

void PowerAdcDriver_GetState(power_adc_driver_state_t *state)
{
    *state = fake_adc_state;
}

void PowerManagement_Update(void)
{
}

uint32_t PowerManagement_GetStatus(power_management_status_t *status)
{
    _Static_assert(sizeof(*status) == sizeof(fake_adc_state), "power status test fixture layout mismatch");
    memcpy(status, &fake_adc_state, sizeof(*status));
    return 1UL;
}

uint32_t StateEstimation_GetWheel(state_estimation_wheel_status_t *status)
{
    *status = fake_encoder_state;
    return 1UL;
}

void MotorDriver_UpdateFaults(void)
{
}

void MotorDriver_GetState(motor_driver_state_t *state)
{
    *state = fake_motor_state;
}

uint8_t MotorDriver_ClearBreakLatch(void)
{
    if (fake_break_clear_allowed == 0U)
    {
        return 0U;
    }
    fake_motor_state.tim1_break_latched = 0U;
    return 1U;
}

command_source_t CommandManagement_GetActiveSource(uint32_t now_ms)
{
    (void)now_ms;
    return fake_active_source;
}

void CommandManagement_SetMotionGate(uint8_t allowed, uint32_t decision_generation)
{
    if ((int32_t)(decision_generation - fake_gate_generation) < 0
        || (decision_generation == fake_gate_generation && fake_gate_allowed == ((allowed != 0U) ? 1U : 0U)))
    {
        return;
    }
    fake_gate_generation = decision_generation;
    fake_gate_allowed    = (allowed != 0U) ? 1U : 0U;
    if (fake_gate_allowed == 0U)
    {
        fake_gate_closed_count++;
    }
}

uint8_t MotorHardwareLayout_MotorEnabled(motor_id_t motor)
{
    return (motor == MOTOR_ID_M2 || motor == MOTOR_ID_M3) ? 1U : 0U;
}

static void require_int(int condition, const char *message)
{
    if (condition == 0)
    {
        (void)printf("FAIL: %s\n", message);
        exit(1);
    }
}

static void synchronize_command_gate(void)
{
    safety_management_status_t state;

    (void)SafetyManagement_GetStatus(&state);
    CommandManagement_SetMotionGate(state.motion_allowed, state.gate_decision_generation);
}

static safety_motor_value_t safety_motor_value(void)
{
    safety_motor_value_t value = {0};

    value.tim1_break_latched = fake_motor_state.tim1_break_latched;
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        value.requested_pwm[index] = fake_motor_state.requested_pwm[index];
        value.applied_pwm[index]   = fake_motor_state.applied_pwm[index];
        value.effective_pwm[index] = fake_motor_state.effective_pwm[index];
        value.fault_active[index]  = fake_motor_state.fault_active[index];
        if (MotorHardwareLayout_MotorEnabled((motor_id_t)index) != 0U)
        {
            value.enabled_mask |= (uint8_t)(1U << index);
        }
    }
    return value;
}

static void safety_update(void)
{
    safety_management_input_t input = {0};

    input.now_ms                      = fake_tick_ms;
    input.power.generation            = PowerManagement_GetStatus(&input.power.value);
    input.power.sample_time_ms        = fake_tick_ms;
    input.power.validity              = 1UL;
    input.wheel.value                 = fake_encoder_state;
    input.wheel.generation            = 1UL;
    input.wheel.sample_time_ms        = fake_tick_ms;
    input.wheel.validity              = 1UL;
    input.motor.value                 = safety_motor_value();
    input.motor.generation            = 1UL;
    input.motor.sample_time_ms        = fake_tick_ms;
    input.motor.validity              = 1UL;
    input.command.value.active_source = fake_active_source;
    input.command.generation          = 1UL;
    input.command.sample_time_ms      = fake_tick_ms;
    input.command.validity            = 1UL;
    SystemMonitoring_UpdateTimeouts(fake_tick_ms);
    input.system.generation     = SystemMonitoring_GetStatus(&input.system.value);
    input.system.sample_time_ms = fake_tick_ms;
    input.system.validity       = 1UL;
    SafetyManagement_UpdateWithInput(&input);
    synchronize_command_gate();
}

static safety_clear_result_t safety_clear(uint32_t mask)
{
    safety_clear_input_t       input = {0};
    safety_management_status_t state;
    safety_clear_result_t      result;

    input.wheel.value      = fake_encoder_state;
    input.wheel.generation = 1UL;
    input.wheel.validity   = 1UL;
    input.motor.value      = safety_motor_value();
    input.motor.generation = 1UL;
    input.motor.validity   = 1UL;
    (void)SafetyManagement_GetStatus(&state);
    if ((mask & SYSTEM_ERROR_TIM_BREAK) != 0UL && state.emergency_stop == 0U)
    {
        input.tim_break_clear_succeeded = MotorDriver_ClearBreakLatch();
        input.motor.value               = safety_motor_value();
    }
    result = SafetyManagement_ClearLatchedFaults(mask, &input);

    synchronize_command_gate();
    return result;
}

static void reset_fake_monitor(void)
{
    const safety_management_config_t safety_config = {
        .battery_low_warn_v           = BATTERY_LOW_WARN_V,
        .battery_low_clear_v          = BATTERY_LOW_CLEAR_V,
        .battery_critical_v           = BATTERY_CRITICAL_V,
        .battery_recover_v            = BATTERY_RECOVER_V,
        .battery_critical_debounce_ms = BATTERY_CRITICAL_DEBOUNCE_MS,
        .battery_recover_debounce_ms  = BATTERY_RECOVER_DEBOUNCE_MS,
        .update_period_ms             = CHASSIS_ADC_PERIOD_MS,
        .overcurrent_startup_blank_ms = MOTOR_OVERCURRENT_STARTUP_BLANK_MS,
        .overcurrent_startup_rearm_ms = MOTOR_OVERCURRENT_STARTUP_REARM_MS,
        .battery_low_monitor_enabled  = BATTERY_LOW_MONITOR_ENABLED,
        .overcurrent_fault_enabled    = MOTOR_ADC_OVERCURRENT_FAULT_ENABLED,
        .remote_velocity_requires_imu = 0U,
        .motion_permit_valid_ms       = CHASSIS_ADC_PERIOD_MS * 2U,
        .current_observe_a            = {1.5f, 1.5f, 1.5f, 1.5f},
        .current_fault_a              = {2.5f, 2.5f, 2.5f, 2.5f},
        .current_fault_debounce_ms    = 100U,
    };
    fake_primask                                   = 0U;
    fake_tick_ms                                   = 1000U;
    fake_adc_state                                 = (power_adc_driver_state_t){0};
    fake_encoder_state                             = (state_estimation_wheel_status_t){0};
    fake_motor_state                               = (motor_driver_state_t){0};
    fake_active_source                             = COMMAND_SOURCE_DEBUG;
    fake_gate_allowed                              = 1U;
    fake_gate_closed_count                         = 0U;
    fake_gate_generation                           = 0UL;
    fake_break_clear_allowed                       = 0U;
    fake_adc_state.current_valid                   = 1U;
    fake_adc_state.current_control_valid           = 1U;
    fake_adc_state.current_control_valid_mask      = (uint8_t)((1U << MOTOR_ID_M2) | (1U << MOTOR_ID_M3));
    fake_adc_state.battery_voltage                 = 12.0f;
    fake_adc_state.samples_ready                   = 1U;
    fake_adc_state.raw_sample_count                = 1U;
    fake_adc_state.valid_flags                     = POWER_MANAGEMENT_ADC_VALID_SAMPLES_READY;
    fake_encoder_state.speed_valid_all             = 1U;
    fake_encoder_state.speed_valid[MOTOR_ID_M2]    = 1U;
    fake_encoder_state.speed_valid[MOTOR_ID_M3]    = 1U;
    const system_monitoring_config_t system_config = {
        .task_timeout_ms = {80U, 40U, 40U, 80U, 40U, 40U, 80U, 200U, 400U},
    };
    require_int(SystemMonitoring_Init(&system_config, 0UL) != 0U, "system monitor config accepted");
    require_int(SafetyManagement_Init(&safety_config) != 0U, "safety config accepted");
    synchronize_command_gate();
    require_int(fake_gate_allowed == 0U, "safety initializes motion gate closed");
    fake_gate_closed_count = 0U;
}

static void test_task_timeout_mask_aggregates_only_timed_out_task(void)
{
    safety_management_status_t state;

    reset_fake_monitor();
    SystemMonitoring_Heartbeat(SYSTEM_MONITORING_TASK_IMU, 100U);
    fake_tick_ms = 181U;
    safety_update();
    (void)SafetyManagement_GetStatus(&state);

    require_int(state.task_timeout_mask == (uint16_t)(1U << SYSTEM_MONITORING_TASK_IMU),
                "system monitor exposes only imu timeout bit");
}

static void update_and_advance(uint32_t step_ms)
{
    safety_update();
    fake_tick_ms += step_ms;
}

static void test_adc_overcurrent_does_not_fault_stop_when_software_fault_disabled(void)
{
#if MOTOR_ADC_OVERCURRENT_FAULT_ENABLED == 0U
    safety_management_status_t state;

    reset_fake_monitor();
    fake_motor_state.output_permille[MOTOR_ID_M2] = 50;
    fake_motor_state.effective_pwm[MOTOR_ID_M2]   = 50;
    fake_adc_state.current_a[MOTOR_ID_M2]         = MOTOR_STALL_CURRENT_A + 1.0f;

    for (uint8_t i = 0U; i < MOTOR_OVERCURRENT_DEBOUNCE_COUNT + 3U; ++i)
    {
        update_and_advance(20U);
    }
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_M2_OVERCURRENT) == 0U,
                "disabled ADC software overcurrent does not latch M2 fault");
    require_int(fake_gate_closed_count == 0U, "disabled ADC software overcurrent does not close motion gate");

    fake_tick_ms = 1300U;
    for (uint8_t i = 0U; i < MOTOR_OVERCURRENT_DEBOUNCE_COUNT; ++i)
    {
        update_and_advance(20U);
    }
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_M2_OVERCURRENT) == 0U,
                "disabled ADC software overcurrent stays clear after startup blank");
    require_int(SafetyManagement_IsFaultStop() == 0U,
                "disabled ADC software overcurrent keeps fault stop clear after blank");
    require_int(state.current_observe_over_limit_count[MOTOR_ID_M2] > 0UL,
                "disabled ADC software overcurrent records dry-run observations");
    require_int(state.current_fault_would_latch_count[MOTOR_ID_M2] > 0UL,
                "disabled ADC software overcurrent records would-latch observations");
#endif
}

static void test_adc_overcurrent_faults_when_enabled_and_control_valid(void)
{
#if MOTOR_ADC_OVERCURRENT_FAULT_ENABLED != 0U
    safety_management_status_t state;

    reset_fake_monitor();
    fake_motor_state.output_permille[MOTOR_ID_M2] = 50;
    fake_motor_state.effective_pwm[MOTOR_ID_M2]   = 50;
    fake_adc_state.current_a[MOTOR_ID_M2]         = MOTOR_STALL_CURRENT_A + 1.0f;

    for (uint8_t i = 0U; i < 12U; ++i)
    {
        update_and_advance(20U);
    }
    for (uint8_t i = 0U; i < MOTOR_OVERCURRENT_DEBOUNCE_COUNT; ++i)
    {
        update_and_advance(20U);
    }
    (void)SafetyManagement_GetStatus(&state);

    require_int((state.latched_error_flags & SYSTEM_ERROR_M2_OVERCURRENT) != 0U,
                "enabled ADC software overcurrent latches M2 fault");
    require_int(SafetyManagement_IsFaultStop() != 0U, "enabled ADC software overcurrent requests fault stop");
    require_int(state.current_observe_over_limit_count[MOTOR_ID_M2] > 0UL,
                "enabled ADC software overcurrent records observations");

    fake_adc_state.current_control_valid_mask = 0U;
    fake_adc_state.current_a[MOTOR_ID_M2]     = 0.0f;
    safety_update();
    (void)safety_clear(SYSTEM_ERROR_M2_OVERCURRENT);
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_M2_OVERCURRENT) != 0U,
                "invalid current sample cannot clear overcurrent latch");
#endif
}

static void test_drv_fault_is_not_suppressed_by_startup_blanking(void)
{
    safety_management_status_t state;

    reset_fake_monitor();
    fake_motor_state.output_permille[MOTOR_ID_M2] = 50;
    fake_motor_state.effective_pwm[MOTOR_ID_M2]   = 50;
    fake_motor_state.fault_active[MOTOR_ID_M2]    = 1U;

    safety_update();
    (void)SafetyManagement_GetStatus(&state);

    require_int((state.latched_error_flags & SYSTEM_ERROR_DRV_FAULT) != 0U, "DRV fault latches during startup blank");
    require_int(SafetyManagement_IsFaultStop() != 0U, "DRV fault requests fault stop during startup blank");
}

static void test_tim1_break_latches_fault_stop_and_requires_driver_clear(void)
{
    safety_management_status_t state;

    reset_fake_monitor();
    fake_motor_state.tim1_break_latched = 1U;
    safety_update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_TIM_BREAK) != 0U, "TIM1 break latches a system fault");
    require_int(SafetyManagement_IsFaultStop() != 0U, "TIM1 break requests fault stop");

    (void)safety_clear(SYSTEM_ERROR_TIM_BREAK);
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_TIM_BREAK) != 0U,
                "driver-rejected clear preserves system break fault");

    fake_break_clear_allowed = 1U;
    (void)safety_clear(SYSTEM_ERROR_TIM_BREAK);
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_TIM_BREAK) == 0U,
                "safe driver clear removes system break fault");
}

static void test_adc_overcurrent_output_chatter_does_not_fault_when_software_fault_disabled(void)
{
#if MOTOR_ADC_OVERCURRENT_FAULT_ENABLED == 0U
    safety_management_status_t state;

    reset_fake_monitor();
    fake_adc_state.current_a[MOTOR_ID_M2] = MOTOR_STALL_CURRENT_A + 1.0f;

    for (uint8_t i = 0U; i < MOTOR_OVERCURRENT_DEBOUNCE_COUNT + 20U; ++i)
    {
        fake_motor_state.output_permille[MOTOR_ID_M2] = ((i & 1U) == 0U) ? 1 : 0;
        fake_motor_state.effective_pwm[MOTOR_ID_M2]   = fake_motor_state.output_permille[MOTOR_ID_M2];
        update_and_advance(20U);
    }
    (void)SafetyManagement_GetStatus(&state);

    require_int((state.latched_error_flags & SYSTEM_ERROR_M2_OVERCURRENT) == 0U,
                "disabled ADC software overcurrent ignores output chatter");
    require_int(SafetyManagement_IsFaultStop() == 0U,
                "disabled ADC software overcurrent chatter keeps fault stop clear");
#endif
}

static void test_encoder_feedback_latch_and_safe_clear(void)
{
    safety_management_status_t state;

    reset_fake_monitor();
    SafetyManagement_LatchEncoderFeedbackFault();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_ENCODER_FEEDBACK_LOST) != 0U,
                "encoder feedback latch records dedicated bit");
    require_int(SafetyManagement_IsFaultStop() != 0U, "encoder feedback latch immediately requests fault stop");

    fake_encoder_state.speed_valid[MOTOR_ID_M2] = 0U;
    (void)safety_clear(SYSTEM_ERROR_ENCODER_FEEDBACK_LOST);
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_ENCODER_FEEDBACK_LOST) != 0U,
                "invalid enabled encoder rejects clear");

    fake_encoder_state.speed_valid[MOTOR_ID_M2] = 1U;
    fake_motor_state.requested_pwm[MOTOR_ID_M2] = 1;
    (void)safety_clear(SYSTEM_ERROR_ENCODER_FEEDBACK_LOST);
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_ENCODER_FEEDBACK_LOST) != 0U,
                "nonzero requested pwm rejects clear");

    fake_motor_state.requested_pwm[MOTOR_ID_M2] = 0;
    (void)safety_clear(SYSTEM_ERROR_ENCODER_FEEDBACK_LOST);
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_ENCODER_FEEDBACK_LOST) == 0U,
                "valid stationary feedback fault clears");
    require_int(SafetyManagement_IsFaultStop() == 0U, "safe clear releases fault stop when no other cause remains");
}

static void test_clear_fault_reports_estop_and_business_result(void)
{
    safety_clear_result_t      result;
    safety_management_status_t state;

    reset_fake_monitor();
    SafetyManagement_LatchEncoderFeedbackFault();
    SafetyManagement_SetEmergencyStop(1U);
    result = safety_clear(SYSTEM_ERROR_ENCODER_FEEDBACK_LOST);
    require_int(result.code == SAFETY_CLEAR_RESULT_CONDITION_NOT_CLEARED, "ESTOP blocks CLEAR_FAULT business success");
    require_int((result.remaining_mask & SYSTEM_ERROR_ENCODER_FEEDBACK_LOST) != 0U,
                "blocked clear reports remaining fault mask");
    (void)SafetyManagement_GetStatus(&state);
    require_int(state.last_clear_result.code == SAFETY_CLEAR_RESULT_CONDITION_NOT_CLEARED,
                "Safety Service retains its own last business result");

    SafetyManagement_SetEmergencyStop(0U);
    fake_encoder_state.speed_valid[MOTOR_ID_M2] = 1U;
    fake_encoder_state.speed_valid[MOTOR_ID_M3] = 1U;
    result                                      = safety_clear(SYSTEM_ERROR_ENCODER_FEEDBACK_LOST);
    require_int(result.code == SAFETY_CLEAR_RESULT_APPLIED && result.remaining_mask == 0U,
                "clear reports applied after physical conditions are safe");
}

static void test_battery_warning_hysteresis(void)
{
    safety_management_status_t state;

    reset_fake_monitor();
    fake_adc_state.battery_voltage = 10.49f;
    safety_update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.error_flags & SYSTEM_ERROR_LOW_BATTERY) != 0U, "10.49V sets low battery warning");

    fake_adc_state.battery_voltage = 10.70f;
    safety_update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.error_flags & SYSTEM_ERROR_LOW_BATTERY) != 0U, "warning remains inside hysteresis band");

    fake_adc_state.battery_voltage = 11.01f;
    safety_update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.error_flags & SYSTEM_ERROR_LOW_BATTERY) == 0U, "11.01V clears low battery warning");
}

static void test_battery_critical_timing_and_recovery(void)
{
    safety_management_status_t state;

    reset_fake_monitor();
    fake_adc_state.battery_voltage = 8.99f;
    fake_tick_ms                   = 1000U;
    safety_update();
    fake_tick_ms = 1499U;
    safety_update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) == 0U,
                "critical battery does not latch at 499ms");
    fake_tick_ms = 1500U;
    safety_update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) != 0U, "critical battery latches at 500ms");
    require_int(SafetyManagement_IsFaultStop() != 0U, "critical battery requests fault stop");

    (void)safety_clear(SYSTEM_ERROR_BATTERY_CRITICAL);
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) != 0U,
                "manual clear cannot remove battery critical");

    fake_adc_state.battery_voltage = 9.61f;
    fake_tick_ms                   = 1600U;
    safety_update();
    fake_tick_ms = 3599U;
    safety_update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) != 0U,
                "battery critical remains at 1999ms recovery");
    fake_tick_ms = 3600U;
    safety_update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) == 0U,
                "battery critical auto clears at 2000ms recovery");
    require_int(SafetyManagement_IsFaultStop() == 0U, "battery-only recovery releases fault stop");
}

static void test_invalid_battery_sample_resets_debounce_and_other_fault_survives_recovery(void)
{
    safety_management_status_t state;

    reset_fake_monitor();
    fake_adc_state.battery_voltage = 8.9f;
    fake_tick_ms                   = 1000U;
    safety_update();
    fake_tick_ms                        = 1400U;
    fake_adc_state.invalid_reason_flags = POWER_MANAGEMENT_ADC_INVALID_NO_NEW_SAMPLE;
    safety_update();
    fake_tick_ms                        = 1500U;
    fake_adc_state.invalid_reason_flags = 0UL;
    safety_update();
    fake_tick_ms = 1900U;
    safety_update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) == 0U,
                "invalid sample resets critical debounce");

    fake_tick_ms = 2000U;
    safety_update();
    SafetyManagement_LatchEncoderFeedbackFault();
    fake_adc_state.battery_voltage = 9.7f;
    fake_tick_ms                   = 2100U;
    safety_update();
    fake_tick_ms = 4100U;
    safety_update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) == 0U,
                "battery recovery clears its own critical bit");
    require_int((state.latched_error_flags & SYSTEM_ERROR_ENCODER_FEEDBACK_LOST) != 0U,
                "battery recovery preserves encoder fault");
    require_int(SafetyManagement_IsFaultStop() != 0U, "other fault keeps fault stop active after battery recovery");
}

static void test_motion_permit_lease_refresh_and_generation(void)
{
    safety_capability_permit_t capabilities = {.base_motion     = 1U,
                                               .manual          = 1U,
                                               .remote_velocity = 1U,
                                               .heading_assist  = 1U};
    safety_motion_permit_t     permit;
    uint32_t                   first_generation;

    reset_fake_monitor();
    SafetyManagement_ApplyCapabilityDecision(&capabilities, 1U, 1000U, SAFETY_STATE_STANDBY);
    synchronize_command_gate();
    first_generation = SafetyManagement_GetMotionPermit(&permit);
    require_int(permit.base_motion != 0U && permit.issued_at_ms == 1000U && permit.valid_for_ms == 40U,
                "capability decision publishes a bounded motion lease");
    require_int(fake_gate_allowed != 0U, "selected capability opens the command gate");

    SafetyManagement_ApplyCapabilityDecision(&capabilities, 1U, 1020U, SAFETY_STATE_ACTIVE);
    synchronize_command_gate();
    require_int(SafetyManagement_GetMotionPermit(&permit) == first_generation,
                "lease refresh does not reset motion state when capabilities are unchanged");
    require_int(permit.issued_at_ms == 1020U, "each safety cycle refreshes the permit issue time");

    capabilities.heading_assist = 0U;
    SafetyManagement_ApplyCapabilityDecision(&capabilities, 1U, 1040U, SAFETY_STATE_ACTIVE);
    synchronize_command_gate();
    require_int(SafetyManagement_GetMotionPermit(&permit) != first_generation,
                "capability changes advance the permit generation");
    require_int(permit.heading_assist == 0U, "changed capability is published atomically with generation");
}

int main(void)
{
    test_task_timeout_mask_aggregates_only_timed_out_task();
    test_adc_overcurrent_does_not_fault_stop_when_software_fault_disabled();
    test_adc_overcurrent_faults_when_enabled_and_control_valid();
    test_drv_fault_is_not_suppressed_by_startup_blanking();
    test_tim1_break_latches_fault_stop_and_requires_driver_clear();
    test_adc_overcurrent_output_chatter_does_not_fault_when_software_fault_disabled();
    test_encoder_feedback_latch_and_safe_clear();
    test_clear_fault_reports_estop_and_business_result();
    test_battery_warning_hysteresis();
    test_battery_critical_timing_and_recovery();
    test_invalid_battery_sample_resets_debounce_and_other_fault_survives_recovery();
    test_motion_permit_lease_refresh_and_generation();
    return 0;
}
