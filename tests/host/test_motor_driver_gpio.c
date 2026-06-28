#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "motor_driver.h"
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
static uint8_t pwm_start_count_tim1;
static uint8_t pwm_start_count_tim8;

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
  (void)pin;
  (void)port;
  return GPIO_PIN_SET;
}

void HAL_Delay(uint32_t delay_ms)
{
  (void)delay_ms;
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
  tim1_instance = (TIM_TypeDef){ .ARR = 8399U, .BDTR = TIM_BDTR_MOE };
  tim8_instance = (TIM_TypeDef){ .ARR = 8399U, .BDTR = TIM_BDTR_MOE };
  for (uint8_t i = 0U; i < 16U; ++i)
  {
    gpio_state_c[i] = GPIO_PIN_RESET;
    gpio_state_d[i] = GPIO_PIN_SET;
  }
  pwm_start_count_tim1 = 0U;
  pwm_start_count_tim8 = 0U;
}

static void test_motor_driver_uses_gpio_for_phase(void)
{
  reset_fake_hw();
  MotorDriver_Init();

  require_int(pwm_start_count_tim1 == 4U, "starts TIM1 PWM for EN channels");
  require_int(pwm_start_count_tim8 == 0U, "does not start TIM8 PWM for PH GPIO pins");

  MotorDriver_SetInputPermille(MOTOR_ID_M2, 300, 0);
  require_int(HostTimGetCompare(&htim1, TIM_CHANNEL_2) == 2520U, "M2 forward writes EN PWM");
  require_int(gpio_c_state(M2_IN2_Pin) == GPIO_PIN_SET, "M2 forward drives PH GPIO high");
  require_int(HostTimGetCompare(&htim8, TIM_CHANNEL_2) == 0U, "M2 PH does not write TIM8 CCR");

  MotorDriver_SetInputPermille(MOTOR_ID_M2, 0, 300);
  require_int(HostTimGetCompare(&htim1, TIM_CHANNEL_2) == 2520U, "M2 reverse keeps EN PWM magnitude");
  require_int(gpio_c_state(M2_IN2_Pin) == GPIO_PIN_RESET, "M2 reverse drives PH GPIO low");
  require_int(HostTimGetCompare(&htim8, TIM_CHANNEL_2) == 0U, "M2 reverse does not write TIM8 CCR");

  MotorDriver_SetInputPermille(MOTOR_ID_M1, 300, 0);
  require_int(HostTimGetCompare(&htim1, TIM_CHANNEL_1) == 0U, "disabled M1 keeps EN off");
  require_int(gpio_c_state(M1_IN2_Pin) == GPIO_PIN_RESET, "disabled M1 keeps PH low");
}

int main(void)
{
  test_motor_driver_uses_gpio_for_phase();
  return 0;
}
