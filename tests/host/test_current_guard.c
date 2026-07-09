#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "adc_monitor.h"
#include "bsp_config.h"
#include "current_guard.h"

static void require_int(int condition, const char *message)
{
  if (condition == 0)
  {
    (void)printf("FAIL: %s\n", message);
    exit(1);
  }
}

static adc_monitor_state_t valid_adc_state(void)
{
  adc_monitor_state_t state = {0};
  state.current_valid = 1U;
  state.current_control_valid = 1U;
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    state.current_control_valid_mask |= (uint8_t)(1U << i);
  }
  return state;
}

static void test_observe_only_does_not_change_pwm(void)
{
  adc_monitor_state_t adc = valid_adc_state();
  current_guard_state_t state;
  uint8_t limited = 0U;
  int16_t applied;

  CurrentGuard_Init();
  adc.current_a[MOTOR_ID_M2] = MOTOR_STALL_CURRENT_A + 1.0f;
  applied = CurrentGuard_ApplyMotorLimit(MOTOR_ID_M2, 600, &adc, 1000U, &limited);
  CurrentGuard_GetState(&state);

  require_int(applied == 600, "observe-only keeps requested PWM");
  require_int(limited == 0U, "observe-only does not report applied soft limit");
  require_int(state.observe_over_limit[MOTOR_ID_M2] != 0U, "observe-only records over-limit observation");
  require_int(state.soft_limit_would_apply[MOTOR_ID_M2] == 0U, "disabled soft-limit threshold does not report would-apply");
  require_int(state.soft_limit_applied[MOTOR_ID_M2] == 0U, "observe-only does not apply soft limit");
}

static void test_fault_would_latch_uses_consecutive_samples(void)
{
  adc_monitor_state_t adc = valid_adc_state();
  current_guard_state_t state;

  CurrentGuard_Init();
  adc.current_a[MOTOR_ID_M2] = MOTOR_STALL_CURRENT_A + 1.0f;
  for (uint8_t i = 0U; i < (uint8_t)(MOTOR_OVERCURRENT_DEBOUNCE_COUNT - 1U); ++i)
  {
    (void)CurrentGuard_ApplyMotorLimit(MOTOR_ID_M2, 600, &adc, 1000U + i, 0);
  }
  CurrentGuard_GetState(&state);
  require_int(state.fault_would_latch[MOTOR_ID_M2] == 0U, "guard does not latch before debounce threshold");

  adc.current_a[MOTOR_ID_M2] = 0.0f;
  (void)CurrentGuard_ApplyMotorLimit(MOTOR_ID_M2, 600, &adc, 2000U, 0);

  adc.current_a[MOTOR_ID_M2] = MOTOR_STALL_CURRENT_A + 1.0f;
  (void)CurrentGuard_ApplyMotorLimit(MOTOR_ID_M2, 600, &adc, 3000U, 0);
  CurrentGuard_GetState(&state);
  require_int(state.fault_would_latch[MOTOR_ID_M2] == 0U, "below-threshold sample resets debounce");
}

static void test_invalid_current_never_intervenes(void)
{
  adc_monitor_state_t adc = valid_adc_state();
  current_guard_state_t state;
  uint8_t limited = 99U;
  int16_t applied;

  CurrentGuard_Init();
  adc.current_control_valid = 0U;
  adc.current_control_valid_mask = 0U;
  adc.current_a[MOTOR_ID_M2] = MOTOR_STALL_CURRENT_A + 2.0f;
  applied = CurrentGuard_ApplyMotorLimit(MOTOR_ID_M2, 700, &adc, 1000U, &limited);
  CurrentGuard_GetState(&state);

  require_int(applied == 700, "invalid current keeps requested PWM");
  require_int(limited == 0U, "invalid current clears limited output flag");
  require_int(state.observe_over_limit[MOTOR_ID_M2] == 0U, "invalid current does not observe over-limit");
  require_int(state.control_valid[MOTOR_ID_M2] == 0U, "invalid current reports guard invalid");
}

static void test_soft_limit_scales_when_enabled(void)
{
  adc_monitor_state_t adc = valid_adc_state();
  current_guard_state_t state;
  uint8_t limited = 0U;
  int16_t applied;

  CurrentGuard_Init();
  adc.current_a[MOTOR_ID_M2] = 2.0f;
  applied = CurrentGuard_ApplyMotorLimit(MOTOR_ID_M2, 600, &adc, 1000U, &limited);
  CurrentGuard_GetState(&state);

  require_int(applied == 300, "enabled soft limit scales PWM by current ratio");
  require_int(limited != 0U, "enabled soft limit reports applied limit");
  require_int(state.soft_limit_would_apply[MOTOR_ID_M2] != 0U, "enabled soft limit records would apply");
  require_int(state.soft_limit_applied[MOTOR_ID_M2] != 0U, "enabled soft limit records applied");
}

static void test_disabled_motor_does_not_participate(void)
{
  adc_monitor_state_t adc = valid_adc_state();
  current_guard_state_t state;
  uint8_t limited = 99U;
  int16_t applied;

  CurrentGuard_Init();
  adc.current_a[MOTOR_ID_M1] = MOTOR_STALL_CURRENT_A + 2.0f;
  applied = CurrentGuard_ApplyMotorLimit(MOTOR_ID_M1, 600, &adc, 1000U, &limited);
  CurrentGuard_GetState(&state);

  require_int(applied == 0, "disabled motor output is forced to zero by guard");
  require_int(limited == 0U, "disabled motor clears limited flag");
  require_int(state.observe_over_limit[MOTOR_ID_M1] == 0U, "disabled motor does not observe over-limit");
}

int main(void)
{
#if !defined(CURRENT_GUARD_TEST_SOFT_LIMIT)
  test_observe_only_does_not_change_pwm();
  test_fault_would_latch_uses_consecutive_samples();
#endif
  test_invalid_current_never_intervenes();
#if defined(CURRENT_GUARD_TEST_SOFT_LIMIT)
  test_soft_limit_scales_when_enabled();
#endif
  test_disabled_motor_does_not_participate();
  return 0;
}
