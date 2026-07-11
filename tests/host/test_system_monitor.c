#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "adc_monitor.h"
#include "bsp_config.h"
#include "chassis_layout.h"
#include "control_manager.h"
#include "encoder_driver.h"
#include "system_monitor.h"

static uint32_t fake_primask;
static uint32_t fake_tick_ms;
static adc_monitor_state_t fake_adc_state;
static encoder_state_t fake_encoder_state;
static motor_driver_state_t fake_motor_state;
static uint8_t fake_estop;
static uint8_t fake_fault_stop;
static uint8_t fake_active_source;
static uint8_t fake_set_fault_stop_count;
static uint8_t fake_break_clear_allowed;

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

void AdcMonitor_Update(void)
{
}

void AdcMonitor_GetState(adc_monitor_state_t *state)
{
  *state = fake_adc_state;
}

void EncoderDriver_GetState(encoder_state_t *state)
{
  *state = fake_encoder_state;
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

uint8_t ControlManager_IsEmergencyStop(void)
{
  return fake_estop;
}

uint8_t ControlManager_IsFaultStop(void)
{
  return fake_fault_stop;
}

uint8_t ControlManager_GetActiveSource(void)
{
  return fake_active_source;
}

void ControlManager_SetFaultStop(uint8_t enabled)
{
  fake_fault_stop = (enabled != 0U) ? 1U : 0U;
  fake_set_fault_stop_count++;
}

uint8_t ChassisLayout_MotorEnabled(motor_id_t motor)
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
  fake_primask = 0U;
  fake_tick_ms = 1000U;
  fake_adc_state = (adc_monitor_state_t){0};
  fake_encoder_state = (encoder_state_t){0};
  fake_motor_state = (motor_driver_state_t){0};
  fake_estop = 0U;
  fake_fault_stop = 0U;
  fake_active_source = CONTROL_SOURCE_DEBUG;
  fake_set_fault_stop_count = 0U;
  fake_break_clear_allowed = 0U;
  fake_adc_state.current_valid = 1U;
  fake_adc_state.current_control_valid = 1U;
  fake_adc_state.current_control_valid_mask = (uint8_t)((1U << MOTOR_ID_M2) | (1U << MOTOR_ID_M3));
  fake_adc_state.battery_voltage = 12.0f;
  fake_adc_state.samples_ready = 1U;
  fake_adc_state.raw_sample_count = 1U;
  fake_adc_state.valid_flags = ADC_MONITOR_VALID_SAMPLES_READY;
  fake_encoder_state.speed_valid_all = 1U;
  fake_encoder_state.speed_valid[MOTOR_ID_M2] = 1U;
  fake_encoder_state.speed_valid[MOTOR_ID_M3] = 1U;
  SystemMonitor_Init();
}

static void update_and_advance(uint32_t step_ms)
{
  SystemMonitor_Update();
  fake_tick_ms += step_ms;
}

static void test_adc_overcurrent_does_not_fault_stop_when_software_fault_disabled(void)
{
#if MOTOR_ADC_OVERCURRENT_FAULT_ENABLED == 0U
  system_monitor_state_t state;

  reset_fake_monitor();
  fake_motor_state.output_permille[MOTOR_ID_M2] = 50;
  fake_motor_state.effective_pwm[MOTOR_ID_M2] = 50;
  fake_adc_state.current_a[MOTOR_ID_M2] = MOTOR_STALL_CURRENT_A + 1.0f;

  for (uint8_t i = 0U; i < MOTOR_OVERCURRENT_DEBOUNCE_COUNT + 3U; ++i)
  {
    update_and_advance(20U);
  }
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_M2_OVERCURRENT) == 0U,
              "disabled ADC software overcurrent does not latch M2 fault");
  require_int(fake_set_fault_stop_count == 0U, "disabled ADC software overcurrent does not request fault stop");

  fake_tick_ms = 1300U;
  for (uint8_t i = 0U; i < MOTOR_OVERCURRENT_DEBOUNCE_COUNT; ++i)
  {
    update_and_advance(20U);
  }
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_M2_OVERCURRENT) == 0U,
              "disabled ADC software overcurrent stays clear after startup blank");
  require_int(fake_fault_stop == 0U, "disabled ADC software overcurrent keeps fault stop clear after blank");
  require_int(state.current_observe_over_limit_count[MOTOR_ID_M2] > 0UL,
              "disabled ADC software overcurrent records dry-run observations");
  require_int(state.current_fault_would_latch_count[MOTOR_ID_M2] > 0UL,
              "disabled ADC software overcurrent records would-latch observations");
#endif
}

static void test_adc_overcurrent_faults_when_enabled_and_control_valid(void)
{
#if MOTOR_ADC_OVERCURRENT_FAULT_ENABLED != 0U
  system_monitor_state_t state;

  reset_fake_monitor();
  fake_motor_state.output_permille[MOTOR_ID_M2] = 50;
  fake_motor_state.effective_pwm[MOTOR_ID_M2] = 50;
  fake_adc_state.current_a[MOTOR_ID_M2] = MOTOR_STALL_CURRENT_A + 1.0f;

  for (uint8_t i = 0U; i < 12U; ++i)
  {
    update_and_advance(20U);
  }
  for (uint8_t i = 0U; i < MOTOR_OVERCURRENT_DEBOUNCE_COUNT; ++i)
  {
    update_and_advance(20U);
  }
  SystemMonitor_GetState(&state);

  require_int((state.latched_error_flags & SYSTEM_ERROR_M2_OVERCURRENT) != 0U,
              "enabled ADC software overcurrent latches M2 fault");
  require_int(fake_fault_stop != 0U, "enabled ADC software overcurrent requests fault stop");
  require_int(state.current_observe_over_limit_count[MOTOR_ID_M2] > 0UL,
              "enabled ADC software overcurrent records observations");
#endif
}

static void test_drv_fault_is_not_suppressed_by_startup_blanking(void)
{
  system_monitor_state_t state;

  reset_fake_monitor();
  fake_motor_state.output_permille[MOTOR_ID_M2] = 50;
  fake_motor_state.effective_pwm[MOTOR_ID_M2] = 50;
  fake_motor_state.fault_active[MOTOR_ID_M2] = 1U;

  SystemMonitor_Update();
  SystemMonitor_GetState(&state);

  require_int((state.latched_error_flags & SYSTEM_ERROR_DRV_FAULT) != 0U,
              "DRV fault latches during startup blank");
  require_int(fake_fault_stop != 0U, "DRV fault requests fault stop during startup blank");
}

static void test_tim1_break_latches_fault_stop_and_requires_driver_clear(void)
{
  system_monitor_state_t state;

  reset_fake_monitor();
  fake_motor_state.tim1_break_latched = 1U;
  SystemMonitor_Update();
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_TIM_BREAK) != 0U,
              "TIM1 break latches a system fault");
  require_int(fake_fault_stop != 0U, "TIM1 break requests fault stop");

  SystemMonitor_ClearLatchedFaults(SYSTEM_ERROR_TIM_BREAK);
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_TIM_BREAK) != 0U,
              "driver-rejected clear preserves system break fault");

  fake_break_clear_allowed = 1U;
  SystemMonitor_ClearLatchedFaults(SYSTEM_ERROR_TIM_BREAK);
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_TIM_BREAK) == 0U,
              "safe driver clear removes system break fault");
}

static void test_adc_overcurrent_output_chatter_does_not_fault_when_software_fault_disabled(void)
{
#if MOTOR_ADC_OVERCURRENT_FAULT_ENABLED == 0U
  system_monitor_state_t state;

  reset_fake_monitor();
  fake_adc_state.current_a[MOTOR_ID_M2] = MOTOR_STALL_CURRENT_A + 1.0f;

  for (uint8_t i = 0U; i < MOTOR_OVERCURRENT_DEBOUNCE_COUNT + 20U; ++i)
  {
    fake_motor_state.output_permille[MOTOR_ID_M2] = ((i & 1U) == 0U) ? 1 : 0;
    fake_motor_state.effective_pwm[MOTOR_ID_M2] = fake_motor_state.output_permille[MOTOR_ID_M2];
    update_and_advance(20U);
  }
  SystemMonitor_GetState(&state);

  require_int((state.latched_error_flags & SYSTEM_ERROR_M2_OVERCURRENT) == 0U,
              "disabled ADC software overcurrent ignores output chatter");
  require_int(fake_fault_stop == 0U, "disabled ADC software overcurrent chatter keeps fault stop clear");
#endif
}

static void test_encoder_feedback_latch_and_safe_clear(void)
{
  system_monitor_state_t state;

  reset_fake_monitor();
  SystemMonitor_LatchEncoderFeedbackFault();
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_ENCODER_FEEDBACK_LOST) != 0U,
              "encoder feedback latch records dedicated bit");
  require_int(fake_fault_stop != 0U, "encoder feedback latch immediately requests fault stop");

  fake_encoder_state.speed_valid[MOTOR_ID_M2] = 0U;
  SystemMonitor_ClearLatchedFaults(SYSTEM_ERROR_ENCODER_FEEDBACK_LOST);
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_ENCODER_FEEDBACK_LOST) != 0U,
              "invalid enabled encoder rejects clear");

  fake_encoder_state.speed_valid[MOTOR_ID_M2] = 1U;
  fake_motor_state.requested_pwm[MOTOR_ID_M2] = 1;
  SystemMonitor_ClearLatchedFaults(SYSTEM_ERROR_ENCODER_FEEDBACK_LOST);
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_ENCODER_FEEDBACK_LOST) != 0U,
              "nonzero requested pwm rejects clear");

  fake_motor_state.requested_pwm[MOTOR_ID_M2] = 0;
  SystemMonitor_ClearLatchedFaults(SYSTEM_ERROR_ENCODER_FEEDBACK_LOST);
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_ENCODER_FEEDBACK_LOST) == 0U,
              "valid stationary feedback fault clears");
  require_int(fake_fault_stop == 0U, "safe clear releases fault stop when no other cause remains");
}

static void test_battery_warning_hysteresis(void)
{
  system_monitor_state_t state;

  reset_fake_monitor();
  fake_adc_state.battery_voltage = 10.49f;
  SystemMonitor_Update();
  SystemMonitor_GetState(&state);
  require_int((state.error_flags & SYSTEM_ERROR_LOW_BATTERY) != 0U,
              "10.49V sets low battery warning");

  fake_adc_state.battery_voltage = 10.70f;
  SystemMonitor_Update();
  SystemMonitor_GetState(&state);
  require_int((state.error_flags & SYSTEM_ERROR_LOW_BATTERY) != 0U,
              "warning remains inside hysteresis band");

  fake_adc_state.battery_voltage = 11.01f;
  SystemMonitor_Update();
  SystemMonitor_GetState(&state);
  require_int((state.error_flags & SYSTEM_ERROR_LOW_BATTERY) == 0U,
              "11.01V clears low battery warning");
}

static void test_battery_critical_timing_and_recovery(void)
{
  system_monitor_state_t state;

  reset_fake_monitor();
  fake_adc_state.battery_voltage = 8.99f;
  fake_tick_ms = 1000U;
  SystemMonitor_Update();
  fake_tick_ms = 1499U;
  SystemMonitor_Update();
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) == 0U,
              "critical battery does not latch at 499ms");
  fake_tick_ms = 1500U;
  SystemMonitor_Update();
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) != 0U,
              "critical battery latches at 500ms");
  require_int(fake_fault_stop != 0U, "critical battery requests fault stop");

  SystemMonitor_ClearLatchedFaults(SYSTEM_ERROR_BATTERY_CRITICAL);
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) != 0U,
              "manual clear cannot remove battery critical");

  fake_adc_state.battery_voltage = 9.61f;
  fake_tick_ms = 1600U;
  SystemMonitor_Update();
  fake_tick_ms = 3599U;
  SystemMonitor_Update();
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) != 0U,
              "battery critical remains at 1999ms recovery");
  fake_tick_ms = 3600U;
  SystemMonitor_Update();
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) == 0U,
              "battery critical auto clears at 2000ms recovery");
  require_int(fake_fault_stop == 0U, "battery-only recovery releases fault stop");
}

static void test_invalid_battery_sample_resets_debounce_and_other_fault_survives_recovery(void)
{
  system_monitor_state_t state;

  reset_fake_monitor();
  fake_adc_state.battery_voltage = 8.9f;
  fake_tick_ms = 1000U;
  SystemMonitor_Update();
  fake_tick_ms = 1400U;
  fake_adc_state.invalid_reason_flags = ADC_MONITOR_INVALID_NO_NEW_SAMPLE;
  SystemMonitor_Update();
  fake_tick_ms = 1500U;
  fake_adc_state.invalid_reason_flags = 0UL;
  SystemMonitor_Update();
  fake_tick_ms = 1900U;
  SystemMonitor_Update();
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) == 0U,
              "invalid sample resets critical debounce");

  fake_tick_ms = 2000U;
  SystemMonitor_Update();
  SystemMonitor_LatchEncoderFeedbackFault();
  fake_adc_state.battery_voltage = 9.7f;
  fake_tick_ms = 2100U;
  SystemMonitor_Update();
  fake_tick_ms = 4100U;
  SystemMonitor_Update();
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_BATTERY_CRITICAL) == 0U,
              "battery recovery clears its own critical bit");
  require_int((state.latched_error_flags & SYSTEM_ERROR_ENCODER_FEEDBACK_LOST) != 0U,
              "battery recovery preserves encoder fault");
  require_int(fake_fault_stop != 0U, "other fault keeps fault stop active after battery recovery");
}

int main(void)
{
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
