#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "motor_driver.h"
#include "param_store.h"
#include "bsp_config.h"
#include "tim.h"

GPIO_TypeDef GPIOA_Instance = { .id = 0x0A };
GPIO_TypeDef GPIOC_Instance = { .id = 0x0C };
GPIO_TypeDef GPIOD_Instance = { .id = 0x0D };
GPIO_TypeDef GPIOE_Instance = { .id = 0x0E };

static TIM_TypeDef tim1_instance = { .ARR = 8399U, .BDTR = TIM_BDTR_MOE };
static TIM_TypeDef tim8_instance = { .ARR = 8399U, .BDTR = TIM_BDTR_MOE };
TIM_HandleTypeDef htim1 = { .Instance = &tim1_instance };
TIM_HandleTypeDef htim8 = { .Instance = &tim8_instance };

static uint32_t fake_primask;
static GPIO_PinState gpio_state_c[16];
static GPIO_PinState gpio_state_d[16];
static GPIO_PinState gpio_state_e[16];
static uint8_t pwm_start_count_tim1;
static uint8_t pwm_start_count_tim8;
static uint32_t fake_tick_ms;
static uint32_t release_startup_inputs_after_ms;
static uint32_t delayed_ms;

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

static uint8_t pin_index(uint16_t pin)
{
  for (uint8_t i = 0U; i < 16U; ++i)
  {
    if (pin == (uint16_t)(1U << i))
    {
      return i;
    }
  }
  return 0U;
}

void HostTimSetCompare(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t pulse)
{
  if (channel == TIM_CHANNEL_1)
  {
    htim->Instance->CCR1 = pulse;
  }
  else if (channel == TIM_CHANNEL_2)
  {
    htim->Instance->CCR2 = pulse;
  }
  else if (channel == TIM_CHANNEL_3)
  {
    htim->Instance->CCR3 = pulse;
  }
  else if (channel == TIM_CHANNEL_4)
  {
    htim->Instance->CCR4 = pulse;
  }
}

uint32_t HostTimGetCompare(TIM_HandleTypeDef *htim, uint32_t channel)
{
  if (channel == TIM_CHANNEL_1)
  {
    return htim->Instance->CCR1;
  }
  if (channel == TIM_CHANNEL_2)
  {
    return htim->Instance->CCR2;
  }
  if (channel == TIM_CHANNEL_3)
  {
    return htim->Instance->CCR3;
  }
  if (channel == TIM_CHANNEL_4)
  {
    return htim->Instance->CCR4;
  }
  return 0U;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel)
{
  (void)channel;
  if (htim == &htim1)
  {
    pwm_start_count_tim1++;
  }
  else if (htim == &htim8)
  {
    pwm_start_count_tim8++;
  }
  return HAL_OK;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
  if (port == GPIOC)
  {
    gpio_state_c[pin_index(pin)] = state;
  }
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
  if (port == GPIOD)
  {
    return gpio_state_d[pin_index(pin)];
  }
  if (port == GPIOE)
  {
    return gpio_state_e[pin_index(pin)];
  }
  (void)pin;
  (void)port;
  return GPIO_PIN_SET;
}

void HAL_Delay(uint32_t delay_ms)
{
  delayed_ms += delay_ms;
  if (release_startup_inputs_after_ms != 0U && delayed_ms >= release_startup_inputs_after_ms)
  {
    gpio_state_e[pin_index(TIM1_BKIN_Pin)] = GPIO_PIN_SET;
    gpio_state_d[pin_index(M3_FAULT_Pin)] = GPIO_PIN_SET;
  }
}

static void require_int(int condition, const char *message)
{
  if (condition == 0)
  {
    (void)printf("FAIL: %s\n", message);
    exit(1);
  }
}

static GPIO_PinState gpio_c_state(uint16_t pin)
{
  return gpio_state_c[pin_index(pin)];
}

static void reset_fake_hw(void)
{
  fake_primask = 0U;
  fake_tick_ms = 100U;
  tim1_instance = (TIM_TypeDef){ .ARR = 8399U, .BDTR = TIM_BDTR_MOE };
  tim8_instance = (TIM_TypeDef){ .ARR = 8399U, .BDTR = TIM_BDTR_MOE };
  for (uint8_t i = 0U; i < 16U; ++i)
  {
    gpio_state_c[i] = GPIO_PIN_RESET;
    gpio_state_d[i] = GPIO_PIN_SET;
    gpio_state_e[i] = GPIO_PIN_SET;
  }
  pwm_start_count_tim1 = 0U;
  pwm_start_count_tim8 = 0U;
  release_startup_inputs_after_ms = 0U;
  delayed_ms = 0U;
}

static void test_motor_driver_uses_gpio_for_phase(void)
{
  motor_driver_state_t state;

  reset_fake_hw();
  MotorDriver_Init();

  require_int(pwm_start_count_tim1 == 4U, "starts TIM1 PWM for EN channels");
  require_int(pwm_start_count_tim8 == 0U, "does not start TIM8 PWM for PH GPIO pins");

  MotorDriver_SetPermille(MOTOR_ID_M3, 300);
  MotorDriver_GetState(&state);
  require_int(HostTimGetCompare(&htim1, TIM_CHANNEL_3) == 126U, "M3 first forward step ramps EN by 15 permille");
  require_int(gpio_c_state(M3_IN2_Pin) == GPIO_PIN_SET, "M3 forward drives PH GPIO high");
  require_int(HostTimGetCompare(&htim8, TIM_CHANNEL_3) == 0U, "M3 PH does not write TIM8 CCR");
  require_int(state.requested_pwm[MOTOR_ID_M3] == 300, "M3 state records requested signed PWM");
  require_int(state.applied_pwm[MOTOR_ID_M3] == 15, "M3 state records ramped signed PWM");
  require_int(state.effective_pwm[MOTOR_ID_M3] == 15, "M3 effective PWM mirrors applied when outputs active");
  require_int(state.output_permille[MOTOR_ID_M3] == 15, "legacy output mirrors applied signed PWM");
  require_int(state.phase[MOTOR_ID_M3] == MOTOR_DRIVER_PHASE_RUN, "M3 enters run phase");

  for (uint8_t i = 0U; i < 19U; ++i)
  {
    MotorDriver_SetPermille(MOTOR_ID_M3, 300);
  }
  MotorDriver_GetState(&state);
  require_int(HostTimGetCompare(&htim1, TIM_CHANNEL_3) == 2520U, "M3 reaches 300 permille after 20 cycles");
  require_int(state.applied_pwm[MOTOR_ID_M3] == 300, "M3 applied PWM reaches target");

  MotorDriver_SetPermille(MOTOR_ID_M1, 300);
  MotorDriver_GetState(&state);
  require_int(HostTimGetCompare(&htim1, TIM_CHANNEL_1) == 0U, "disabled M1 keeps EN off");
  require_int(gpio_c_state(M1_IN2_Pin) == GPIO_PIN_RESET, "disabled M1 keeps PH low");
  require_int(state.output_permille[MOTOR_ID_M1] == 0, "disabled M1 state records off output");
}

static void test_motor_driver_uses_runtime_direction(void)
{
  motor_driver_state_t state;
  param_store_t params;

  reset_fake_hw();
  ParamStore_Defaults(&params);
  params.motor_dir[MOTOR_ID_M3] = -1;
  require_int(ParamStore_Set(&params) != 0U, "runtime motor direction accepted");
  MotorDriver_Init();
  MotorDriver_SetPermille(MOTOR_ID_M3, 300);
  MotorDriver_GetState(&state);
  require_int(state.requested_pwm[MOTOR_ID_M3] == -300,
              "runtime motor direction reverses requested output");
  ParamStore_SetDefaults();
}

static void test_motor_driver_ramps_up_and_down_without_changing_phase(void)
{
  motor_driver_state_t state;

  reset_fake_hw();
  MotorDriver_Init();

  for (uint8_t i = 0U; i < 60U; ++i)
  {
    MotorDriver_SetPermille(MOTOR_ID_M3, 900);
  }
  MotorDriver_GetState(&state);
  require_int(state.applied_pwm[MOTOR_ID_M3] == 900, "M3 reaches 900 after 60 rise cycles");
  require_int(HostTimGetCompare(&htim1, TIM_CHANNEL_3) == 7560U, "M3 900 permille writes expected EN PWM");
  require_int(gpio_c_state(M3_IN2_Pin) == GPIO_PIN_SET, "M3 PH remains high while running forward");

  for (uint8_t i = 0U; i < 35U; ++i)
  {
    MotorDriver_SetPermille(MOTOR_ID_M3, 0);
    MotorDriver_GetState(&state);
    require_int(gpio_c_state(M3_IN2_Pin) == GPIO_PIN_SET, "M3 PH stays high while ramping down to stop");
    require_int(state.applied_pwm[MOTOR_ID_M3] > 0, "M3 still has nonzero PWM before final down cycle");
  }
  MotorDriver_SetPermille(MOTOR_ID_M3, 0);
  MotorDriver_GetState(&state);
  require_int(state.applied_pwm[MOTOR_ID_M3] == 0, "M3 reaches zero after 36 down cycles");
  require_int(HostTimGetCompare(&htim1, TIM_CHANNEL_3) == 0U, "M3 EN is low after normal stop");
  require_int(gpio_c_state(M3_IN2_Pin) == GPIO_PIN_SET, "M3 PH is preserved after normal stop");
  require_int(state.phase[MOTOR_ID_M3] == MOTOR_DRIVER_PHASE_IDLE_BRAKE, "M3 enters idle brake after zero target");
}

static void test_motor_driver_reverses_only_after_brake_and_phase_settle(void)
{
  motor_driver_state_t state;

  reset_fake_hw();
  MotorDriver_Init();

  for (uint8_t i = 0U; i < 20U; ++i)
  {
    MotorDriver_SetPermille(MOTOR_ID_M3, 300);
  }
  MotorDriver_GetState(&state);
  require_int(state.applied_pwm[MOTOR_ID_M3] == 300, "M3 starts at forward 300");
  require_int(gpio_c_state(M3_IN2_Pin) == GPIO_PIN_SET, "M3 starts with forward PH");

  for (uint8_t i = 0U; i < 11U; ++i)
  {
    MotorDriver_SetPermille(MOTOR_ID_M3, -300);
    MotorDriver_GetState(&state);
    require_int(gpio_c_state(M3_IN2_Pin) == GPIO_PIN_SET, "M3 PH does not change while PWM is nonzero");
    require_int(state.phase[MOTOR_ID_M3] == MOTOR_DRIVER_PHASE_RAMP_DOWN, "M3 ramps down before reverse");
  }

  MotorDriver_SetPermille(MOTOR_ID_M3, -300);
  MotorDriver_GetState(&state);
  require_int(state.applied_pwm[MOTOR_ID_M3] == 0, "M3 reaches zero before reverse brake");
  require_int(gpio_c_state(M3_IN2_Pin) == GPIO_PIN_SET, "M3 PH still unchanged at zero");
  require_int(state.phase[MOTOR_ID_M3] == MOTOR_DRIVER_PHASE_REVERSE_BRAKE, "M3 enters reverse brake");

  MotorDriver_SetPermille(MOTOR_ID_M3, -300);
  MotorDriver_GetState(&state);
  require_int(gpio_c_state(M3_IN2_Pin) == GPIO_PIN_SET, "M3 holds old PH for first brake cycle");
  require_int(state.phase[MOTOR_ID_M3] == MOTOR_DRIVER_PHASE_REVERSE_BRAKE, "M3 stays in reverse brake");

  MotorDriver_SetPermille(MOTOR_ID_M3, -300);
  MotorDriver_GetState(&state);
  require_int(gpio_c_state(M3_IN2_Pin) == GPIO_PIN_SET, "M3 holds old PH for second brake cycle");
  require_int(state.phase[MOTOR_ID_M3] == MOTOR_DRIVER_PHASE_PH_SETTLE, "M3 waits for PH switch permission");

  MotorDriver_SetPermille(MOTOR_ID_M3, -300);
  MotorDriver_GetState(&state);
  require_int(gpio_c_state(M3_IN2_Pin) == GPIO_PIN_RESET, "M3 writes new PH only with EN low");
  require_int(state.current_ph_dir[MOTOR_ID_M3] == -1, "M3 records new PH direction");
  require_int(state.applied_pwm[MOTOR_ID_M3] == 0, "M3 keeps EN low when PH changes");
  require_int(state.phase[MOTOR_ID_M3] == MOTOR_DRIVER_PHASE_RAMP_UP, "M3 enters ramp-up after PH settle");

  MotorDriver_SetPermille(MOTOR_ID_M3, -300);
  MotorDriver_GetState(&state);
  require_int(state.applied_pwm[MOTOR_ID_M3] == -15, "M3 ramps up in new direction after settle");
  require_int(HostTimGetCompare(&htim1, TIM_CHANNEL_3) == 126U, "M3 reverse ramp writes first EN step");
}

static void test_motor_driver_serializes_phase_switches(void)
{
  motor_driver_state_t state;

  reset_fake_hw();
  MotorDriver_Init();

  for (uint8_t i = 0U; i < 20U; ++i)
  {
    MotorDriver_SetPermille(MOTOR_ID_M2, 300);
    MotorDriver_SetPermille(MOTOR_ID_M3, 300);
  }
  require_int(gpio_c_state(M2_IN2_Pin) == GPIO_PIN_RESET, "M2 positive command maps to low PH by layout");
  require_int(gpio_c_state(M3_IN2_Pin) == GPIO_PIN_SET, "M3 positive command maps to high PH by layout");

  for (uint8_t i = 0U; i < 14U; ++i)
  {
    MotorDriver_SetPermille(MOTOR_ID_M2, -300);
    MotorDriver_SetPermille(MOTOR_ID_M3, -300);
  }
  MotorDriver_SetPermille(MOTOR_ID_M2, -300);
  MotorDriver_GetState(&state);
  require_int(state.applied_pwm[MOTOR_ID_M3] == 0, "M3 is ready to switch PH");
  require_int(gpio_c_state(M2_IN2_Pin) == GPIO_PIN_SET, "M2 switches PH first");
  require_int(gpio_c_state(M3_IN2_Pin) == GPIO_PIN_SET, "M3 PH has not switched in same control round");

  for (uint8_t i = 0U; i < 4U; ++i)
  {
    MotorDriver_SetPermille(MOTOR_ID_M3, -300);
  }
  require_int(gpio_c_state(M3_IN2_Pin) == GPIO_PIN_RESET, "M3 switches PH after serialized gap");
}

static void test_motor_driver_emergency_stop_preserves_phase_and_restarts_safely(void)
{
  motor_driver_state_t state;

  reset_fake_hw();
  MotorDriver_Init();

  for (uint8_t i = 0U; i < 20U; ++i)
  {
    MotorDriver_SetPermille(MOTOR_ID_M3, 300);
  }
  require_int(gpio_c_state(M3_IN2_Pin) == GPIO_PIN_SET, "M3 forward PH before emergency stop");

  MotorDriver_StopAll(MOTOR_STOP_LOW_SIDE_BRAKE);
  MotorDriver_GetState(&state);
  require_int(state.requested_pwm[MOTOR_ID_M3] == 0, "M3 emergency stop clears requested PWM");
  require_int(state.applied_pwm[MOTOR_ID_M3] == 0, "M3 emergency stop clears applied PWM");
  require_int(HostTimGetCompare(&htim1, TIM_CHANNEL_3) == 0U, "M3 emergency stop immediately drives EN low");
  require_int(gpio_c_state(M3_IN2_Pin) == GPIO_PIN_SET, "M3 emergency stop preserves PH");

  MotorDriver_SetPermille(MOTOR_ID_M3, -300);
  MotorDriver_GetState(&state);
  require_int(state.applied_pwm[MOTOR_ID_M3] == 0, "M3 restart writes PH before PWM");
  require_int(gpio_c_state(M3_IN2_Pin) == GPIO_PIN_RESET, "M3 restart sets requested PH with EN low");
  require_int(state.phase[MOTOR_ID_M3] == MOTOR_DRIVER_PHASE_RAMP_UP, "M3 restart waits one call before PWM");

  MotorDriver_SetPermille(MOTOR_ID_M3, -300);
  MotorDriver_GetState(&state);
  require_int(state.applied_pwm[MOTOR_ID_M3] == -15, "M3 restart ramps PWM after PH settle");
}

static void test_motor_driver_effective_pwm_and_fault_edges(void)
{
  motor_driver_state_t state;

  reset_fake_hw();
  MotorDriver_Init();

  for (uint8_t i = 0U; i < 20U; ++i)
  {
    MotorDriver_SetPermille(MOTOR_ID_M3, 300);
  }
  MotorDriver_GetState(&state);
  require_int(state.applied_pwm[MOTOR_ID_M3] == 300, "M3 applied before break");
  require_int(state.effective_pwm[MOTOR_ID_M3] == 300, "M3 effective before break");

  tim1_instance.BDTR &= ~TIM_BDTR_MOE;
  fake_tick_ms += 5U;
  MotorDriver_GetState(&state);
  require_int(state.applied_pwm[MOTOR_ID_M3] == 300, "M3 applied preserved while break active");
  require_int(state.effective_pwm[MOTOR_ID_M3] == 0, "M3 effective zero while TIM1 MOE inactive");

  tim1_instance.BDTR |= TIM_BDTR_MOE;
  gpio_state_d[14] = GPIO_PIN_RESET;
  fake_tick_ms += 5U;
  MotorDriver_UpdateFaults();
  MotorDriver_GetState(&state);
  require_int(state.fault_active[MOTOR_ID_M2] != 0U, "logical M2 fault follows crossed PD14 pin");
  require_int(state.fault_edge_count[MOTOR_ID_M2] == 1U, "fault falling edge counted");
  require_int(state.fault_low_since_ms[MOTOR_ID_M2] != 0U, "fault low timestamp captured");

  gpio_state_d[14] = GPIO_PIN_SET;
  fake_tick_ms += 5U;
  MotorDriver_UpdateFaults();
  MotorDriver_GetState(&state);
  require_int(state.fault_active[MOTOR_ID_M2] == 0U, "fault release sampled");
  require_int(state.fault_edge_count[MOTOR_ID_M2] == 2U, "fault rising edge counted");
  require_int(state.fault_low_since_ms[MOTOR_ID_M2] == 0U, "fault low timestamp cleared");
  require_int(state.fault_last_change_ms[MOTOR_ID_M2] != 0U, "fault last change timestamp captured");
}

static void test_tim1_break_latches_until_safe_manual_clear(void)
{
  motor_driver_state_t state;

  reset_fake_hw();
  MotorDriver_Init();
  for (uint8_t i = 0U; i < 20U; ++i)
  {
    MotorDriver_SetPermille(MOTOR_ID_M3, 300);
  }
  tim1_instance.SR |= TIM_FLAG_BREAK;
  fake_tick_ms = 1234U;
  MotorDriver_OnTim1BreakFromIsr();
  MotorDriver_GetState(&state);
  require_int(state.tim1_break_latched != 0U, "TIM1 break remains latched after pulse");
  require_int(state.tim1_break_last_ms == 1234U, "TIM1 break records ISR timestamp");
  require_int((tim1_instance.BDTR & TIM_BDTR_MOE) == 0U, "TIM1 break clears MOE");
  require_int(tim1_instance.CCR1 == 0U && tim1_instance.CCR2 == 0U &&
              tim1_instance.CCR3 == 0U && tim1_instance.CCR4 == 0U,
              "TIM1 break clears every PWM compare");
  require_int(state.effective_pwm[MOTOR_ID_M3] == 0, "TIM1 break forces effective PWM to zero");
  require_int(MotorDriver_HasFault() != 0U, "latched break is a motor fault");

  gpio_state_e[pin_index(TIM1_BKIN_Pin)] = GPIO_PIN_RESET;
  require_int(MotorDriver_ClearBreakLatch() == 0U, "active BKIN blocks manual clear");
  MotorDriver_GetState(&state);
  require_int(state.tim1_break_latched != 0U, "failed clear preserves break latch");

  gpio_state_e[pin_index(TIM1_BKIN_Pin)] = GPIO_PIN_SET;
  require_int(MotorDriver_ClearBreakLatch() == 1U, "released BKIN permits safe manual clear");
  MotorDriver_GetState(&state);
  require_int(state.tim1_break_latched == 0U, "safe clear removes break latch");
  require_int((tim1_instance.BDTR & TIM_BDTR_MOE) != 0U, "safe clear re-enables TIM1 outputs");
}

static void test_tim8_break_is_timestamped_redundant_diagnostic_only(void)
{
  motor_driver_state_t state;

  reset_fake_hw();
  MotorDriver_Init();
  fake_tick_ms = 2345U;
  tim8_instance.SR |= TIM_FLAG_BREAK;
  MotorDriver_UpdateFaults();
  MotorDriver_GetState(&state);

  require_int(state.tim8_break_count == 1UL, "TIM8 break diagnostic is counted");
  require_int(state.tim8_break_last_ms == 2345U, "TIM8 break diagnostic is timestamped");
  require_int(state.tim1_break_latched == 0U, "TIM8 shared-net diagnostic does not create second latch");
}

static void test_pre_wake_tim1_break_is_cleared_after_inputs_stabilize(void)
{
  motor_driver_state_t state;

  reset_fake_hw();
  tim1_instance.SR |= TIM_FLAG_BREAK;
  MotorDriver_Init();
  MotorDriver_GetState(&state);
  require_int(state.tim1_break_latched == 0U,
              "pre-wake TIM1 break is not treated as a runtime fault after stable high inputs");
  require_int(state.startup_qualified != 0U,
              "stable BKIN and nFAULT inputs qualify startup");
}

static void test_startup_low_bkin_latches_break_and_keeps_outputs_disabled(void)
{
  motor_driver_state_t state;

  reset_fake_hw();
  gpio_state_e[pin_index(TIM1_BKIN_Pin)] = GPIO_PIN_RESET;
  MotorDriver_Init();
  MotorDriver_GetState(&state);
  require_int(state.tim1_break_latched != 0U,
              "BKIN held low through startup qualification latches break");
  require_int(state.startup_qualified == 0U,
              "low BKIN cannot qualify startup");
  require_int(state.break_origin == MOTOR_BREAK_ORIGIN_STARTUP_TIMEOUT,
              "startup qualification timeout is recorded as break origin");
  require_int((tim1_instance.BDTR & TIM_BDTR_MOE) == 0U,
              "failed startup qualification keeps main output disabled");
}

static void test_startup_transient_must_recover_then_remain_stable(void)
{
  motor_driver_state_t state;

  reset_fake_hw();
  gpio_state_e[pin_index(TIM1_BKIN_Pin)] = GPIO_PIN_RESET;
  release_startup_inputs_after_ms = DRV8874_WAKE_DELAY_MS + 3U;
  MotorDriver_Init();
  MotorDriver_GetState(&state);
  require_int(state.startup_qualified != 0U && state.tim1_break_latched == 0U,
              "brief wake transient is accepted only after a stable-high window");
}

int main(void)
{
  test_motor_driver_uses_runtime_direction();
  test_motor_driver_uses_gpio_for_phase();
  test_motor_driver_ramps_up_and_down_without_changing_phase();
  test_motor_driver_reverses_only_after_brake_and_phase_settle();
  test_motor_driver_serializes_phase_switches();
  test_motor_driver_emergency_stop_preserves_phase_and_restarts_safely();
  test_motor_driver_effective_pwm_and_fault_edges();
  test_tim8_break_is_timestamped_redundant_diagnostic_only();
  test_tim1_break_latches_until_safe_manual_clear();
  test_pre_wake_tim1_break_is_cleared_after_inputs_stabilize();
  test_startup_low_bkin_latches_break_and_keeps_outputs_disabled();
  test_startup_transient_must_recover_then_remain_stable();
  return 0;
}
