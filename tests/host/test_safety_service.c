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
    (void)decision_generation;
    fake_gate_allowed = (allowed != 0U) ? 1U : 0U;
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
    fake_break_clear_allowed                       = 0U;
    fake_adc_state.current_valid                   = 1U;
    fake_adc_state.current_control_valid           = 1U;
    fake_adc_state.current_control_valid_mask      = (uint8_t)((1U << MOTOR_ID_M2) | (1U << MOTOR_ID_M3));
    fake_adc_state.battery_voltage                 = 12.0f;
    fake_adc_state.samples_ready                   = 1U;
    fake_adc_state.raw_sample_count                = 1U;
    fake_adc_state.valid_flags                     = POWER_ADC_DRIVER_VALID_SAMPLES_READY;
    fake_encoder_state.speed_valid_all             = 1U;
    fake_encoder_state.speed_valid[MOTOR_ID_M2]    = 1U;
    fake_encoder_state.speed_valid[MOTOR_ID_M3]    = 1U;
    const system_monitoring_config_t system_config = {
        .task_timeout_ms = {80U, 40U, 40U, 80U, 40U, 40U, 80U, 200U, 400U},
    };
    require_int(SystemMonitoring_Init(&system_config, 0UL) != 0U, "system monitor config accepted");
    require_int(SafetyManagement_Init(&safety_config) != 0U, "safety config accepted");
}

static void test_task_timeout_mask_aggregates_only_timed_out_task(void)
{
    safety_management_status_t state;

    reset_fake_monitor();
    SystemMonitoring_Heartbeat(SYSTEM_MONITORING_TASK_IMU, 100U);
    fake_tick_ms = 181U;
    SafetyManagement_Update();
    (void)SafetyManagement_GetStatus(&state);

    require_int(state.task_timeout_mask == (uint16_t)(1U << SYSTEM_MONITORING_TASK_IMU),
                "system monitor exposes only imu timeout bit");
}

static void update_and_advance(uint32_t step_ms)
{
    SafetyManagement_Update();
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
    SafetyManagement_Update();
    SafetyManagement_ClearLatchedFaults(SYSTEM_ERROR_M2_OVERCURRENT);
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

    SafetyManagement_Update();
    (void)SafetyManagement_GetStatus(&state);

    require_int((state.latched_error_flags & SYSTEM_ERROR_DRV_FAULT) != 0U, "DRV fault latches during startup blank");
    require_int(SafetyManagement_IsFaultStop() != 0U, "DRV fault requests fault stop during startup blank");
}

static void test_tim1_break_latches_fault_stop_and_requires_driver_clear(void)
{
    safety_management_status_t state;

    reset_fake_monitor();
    fake_motor_state.tim1_break_latched = 1U;
    SafetyManagement_Update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_TIM_BREAK) != 0U, "TIM1 break latches a system fault");
    require_int(SafetyManagement_IsFaultStop() != 0U, "TIM1 break requests fault stop");

    SafetyManagement_ClearLatchedFaults(SYSTEM_ERROR_TIM_BREAK);
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_TIM_BREAK) != 0U,
                "driver-rejected clear preserves system break fault");

    fake_break_clear_allowed = 1U;
    SafetyManagement_ClearLatchedFaults(SYSTEM_ERROR_TIM_BREAK);
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
    SafetyManagement_ClearLatchedFaults(SYSTEM_ERROR_ENCODER_FEEDBACK_LOST);
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_ENCODER_FEEDBACK_LOST) != 0U,
                "invalid enabled encoder rejects clear");

    fake_encoder_state.speed_valid[MOTOR_ID_M2] = 1U;
    fake_motor_state.requested_pwm[MOTOR_ID_M2] = 1;
    SafetyManagement_ClearLatchedFaults(SYSTEM_ERROR_ENCODER_FEEDBACK_LOST);
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_ENCODER_FEEDBACK_LOST) != 0U,
                "nonzero requested pwm rejects clear");

    fake_motor_state.requested_pwm[MOTOR_ID_M2] = 0;
    SafetyManagement_ClearLatchedFaults(SYSTEM_ERROR_ENCODER_FEEDBACK_LOST);
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_ENCODER_FEEDBACK_LOST) == 0U,
                "valid stationary feedback fault clears");
    require_int(SafetyManagement_IsFaultStop() == 0U, "safe clear releases fault stop when no other cause remains");
}

static void test_battery_warning_hysteresis(void)
{
    safety_management_status_t state;

    reset_fake_monitor();
    fake_adc_state.battery_voltage = 10.49f;
    SafetyManagement_Update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.error_flags & SYSTEM_ERROR_LOW_BATTERY) != 0U, "10.49V sets low battery warning");

    fake_adc_state.battery_voltage = 10.70f;
    SafetyManagement_Update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.error_flags & SYSTEM_ERROR_LOW_BATTERY) != 0U, "warning remains inside hysteresis band");

    fake_adc_state.battery_voltage = 11.01f;
    SafetyManagement_Update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.error_flags & SYSTEM_ERROR_LOW_BATTERY) == 0U, "11.01V clears low battery warning");
}

static void test_battery_critical_timing_and_recovery(void)
{
    safety_management_status_t state;

    reset_fake_monitor();
    fake_adc_state.battery_voltage = 8.99f;
    fake_tick_ms                   = 1000U;
    SafetyManagement_Update();
    fake_tick_ms = 1499U;
    SafetyManagement_Update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) == 0U,
                "critical battery does not latch at 499ms");
    fake_tick_ms = 1500U;
    SafetyManagement_Update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) != 0U, "critical battery latches at 500ms");
    require_int(SafetyManagement_IsFaultStop() != 0U, "critical battery requests fault stop");

    SafetyManagement_ClearLatchedFaults(SYSTEM_ERROR_BATTERY_CRITICAL);
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) != 0U,
                "manual clear cannot remove battery critical");

    fake_adc_state.battery_voltage = 9.61f;
    fake_tick_ms                   = 1600U;
    SafetyManagement_Update();
    fake_tick_ms = 3599U;
    SafetyManagement_Update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) != 0U,
                "battery critical remains at 1999ms recovery");
    fake_tick_ms = 3600U;
    SafetyManagement_Update();
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
    SafetyManagement_Update();
    fake_tick_ms                        = 1400U;
    fake_adc_state.invalid_reason_flags = POWER_ADC_DRIVER_INVALID_NO_NEW_SAMPLE;
    SafetyManagement_Update();
    fake_tick_ms                        = 1500U;
    fake_adc_state.invalid_reason_flags = 0UL;
    SafetyManagement_Update();
    fake_tick_ms = 1900U;
    SafetyManagement_Update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) == 0U,
                "invalid sample resets critical debounce");

    fake_tick_ms = 2000U;
    SafetyManagement_Update();
    SafetyManagement_LatchEncoderFeedbackFault();
    fake_adc_state.battery_voltage = 9.7f;
    fake_tick_ms                   = 2100U;
    SafetyManagement_Update();
    fake_tick_ms = 4100U;
    SafetyManagement_Update();
    (void)SafetyManagement_GetStatus(&state);
    require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) == 0U,
                "battery recovery clears its own critical bit");
    require_int((state.latched_error_flags & SYSTEM_ERROR_ENCODER_FEEDBACK_LOST) != 0U,
                "battery recovery preserves encoder fault");
    require_int(SafetyManagement_IsFaultStop() != 0U, "other fault keeps fault stop active after battery recovery");
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
    test_battery_warning_hysteresis();
    test_battery_critical_timing_and_recovery();
    test_invalid_battery_sample_resets_debounce_and_other_fault_survives_recovery();
    return 0;
}
