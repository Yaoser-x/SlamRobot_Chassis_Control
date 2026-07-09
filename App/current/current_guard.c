#include "current_guard.h"

#include "bsp_config.h"
#include "chassis_layout.h"

static current_guard_state_t current_guard_state;
static uint8_t current_guard_over_limit_debounce[MOTOR_ID_COUNT];

static int16_t CurrentGuard_ClampPermille(int32_t value)
{
  if (value > 1000)
  {
    return 1000;
  }
  if (value < -1000)
  {
    return -1000;
  }
  return (int16_t)value;
}

static uint8_t CurrentGuard_IsControlValid(motor_id_t motor, const adc_monitor_state_t *adc_state)
{
  uint8_t mask;

  if (adc_state == 0 || (uint32_t)motor >= MOTOR_ID_COUNT)
  {
    return 0U;
  }
  mask = (uint8_t)(1U << (uint8_t)motor);
  return (adc_state->current_control_valid != 0U &&
          (adc_state->current_control_valid_mask & mask) != 0U) ? 1U : 0U;
}

void CurrentGuard_Init(void)
{
  current_guard_state = (current_guard_state_t){0};
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    current_guard_over_limit_debounce[i] = 0U;
  }
}

int16_t CurrentGuard_ApplyMotorLimit(motor_id_t motor,
                                     int16_t requested_permille,
                                     const adc_monitor_state_t *adc_state,
                                     uint32_t now_ms,
                                     uint8_t *limited)
{
  uint8_t control_valid;
  uint8_t over_soft_limit = 0U;
  uint8_t over_fault_limit = 0U;
  int16_t applied;

  (void)now_ms;
  if (limited != 0)
  {
    *limited = 0U;
  }
  if ((uint32_t)motor >= MOTOR_ID_COUNT)
  {
    return 0;
  }

  current_guard_state.observe_over_limit[motor] = 0U;
  current_guard_state.soft_limit_would_apply[motor] = 0U;
  current_guard_state.soft_limit_applied[motor] = 0U;
  current_guard_state.fault_would_latch[motor] = 0U;
  current_guard_state.control_valid[motor] = 0U;
  current_guard_state.applied_permille[motor] = requested_permille;

  if (ChassisLayout_MotorEnabled(motor) == 0U)
  {
    current_guard_state.applied_permille[motor] = 0;
    return 0;
  }

  applied = requested_permille;
  control_valid = CurrentGuard_IsControlValid(motor, adc_state);
  current_guard_state.control_valid[motor] = control_valid;
  if (control_valid == 0U || adc_state == 0)
  {
    return applied;
  }

  if (adc_state->current_a[motor] > MOTOR_STALL_CURRENT_A)
  {
    over_fault_limit = 1U;
    current_guard_state.observe_over_limit_count[motor]++;
    current_guard_state.observe_over_limit[motor] = 1U;
    if (current_guard_over_limit_debounce[motor] < MOTOR_OVERCURRENT_DEBOUNCE_COUNT)
    {
      current_guard_over_limit_debounce[motor]++;
    }
  }
  else
  {
    current_guard_over_limit_debounce[motor] = 0U;
  }

  if (MOTOR_CURRENT_LIMIT_A > 0.0f &&
      adc_state->current_a[motor] > MOTOR_CURRENT_LIMIT_A &&
      requested_permille != 0)
  {
    over_soft_limit = 1U;
    current_guard_state.soft_limit_would_apply[motor] = 1U;
  }

  if (over_fault_limit != 0U &&
      current_guard_over_limit_debounce[motor] >= MOTOR_OVERCURRENT_DEBOUNCE_COUNT)
  {
    current_guard_state.fault_would_latch[motor] = 1U;
    current_guard_state.fault_would_latch_count[motor]++;
  }

  if (MOTOR_CURRENT_GUARD_OBSERVE_ONLY == 0U &&
      MOTOR_CURRENT_SOFT_LIMIT_ENABLED != 0U &&
      over_soft_limit != 0U)
  {
    applied = CurrentGuard_ClampPermille((int32_t)((float)requested_permille *
                                                  (MOTOR_CURRENT_LIMIT_A / adc_state->current_a[motor])));
    current_guard_state.soft_limit_applied[motor] = 1U;
    if (limited != 0)
    {
      *limited = 1U;
    }
  }

  current_guard_state.applied_permille[motor] = applied;
  return applied;
}

void CurrentGuard_GetState(current_guard_state_t *state)
{
  if (state != 0)
  {
    *state = current_guard_state;
  }
}
