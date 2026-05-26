#include "led_status.h"

#include "main.h"

static led_status_mode_t led_mode = LED_STATUS_NORMAL;
static uint32_t led_tick_ms;

void LedStatus_Init(void)
{
  led_mode = LED_STATUS_NORMAL;
  led_tick_ms = 0U;
  HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_RESET);
}

void LedStatus_SetMode(led_status_mode_t mode)
{
  led_mode = mode;
}

void LedStatus_TaskStep(uint32_t period_ms)
{
  uint32_t period;
  uint32_t on_time;

  led_tick_ms += period_ms;

  switch (led_mode)
  {
    case LED_STATUS_LOW_BATTERY:
    case LED_STATUS_FAULT:
    case LED_STATUS_ESTOP:
      period = 200U;
      on_time = 100U;
      break;
    case LED_STATUS_UPPER_LINK:
      period = 600U;
      on_time = 300U;
      break;
    case LED_STATUS_NORMAL:
    default:
      period = 1000U;
      on_time = 500U;
      break;
  }

  if (led_tick_ms >= period)
  {
    led_tick_ms = 0U;
  }

  HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, (led_tick_ms < on_time) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
