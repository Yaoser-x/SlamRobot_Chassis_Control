#include "led_status.h"

#include "main.h"

static led_status_mode_t led_mode = LED_STATUS_NORMAL;
static uint32_t led_tick_ms;
static uint32_t led_mode_elapsed_ms;  /* monotonic, reset on SetMode */
static uint32_t led_period;           /* current blink period */
static uint32_t led_on_time;          /* current ON duration within period */

void LedStatus_Init(void)
{
  led_mode = LED_STATUS_NORMAL;
  led_tick_ms = 0U;
  led_mode_elapsed_ms = 0U;
  led_period = 1000U;
  led_on_time = 500U;
  HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_RESET);
}

void LedStatus_SetMode(led_status_mode_t mode)
{
  if (mode != led_mode)
  {
    led_mode = mode;
    led_mode_elapsed_ms = 0U;
    led_tick_ms = 0U;
  }
}

static void LedStatus_SelectParams(void)
{
  switch (led_mode)
  {
    case LED_STATUS_LOW_BATTERY:
    case LED_STATUS_FAULT:
    case LED_STATUS_ESTOP:
      led_period = 200U;
      led_on_time = 100U;
      break;
    case LED_STATUS_UPPER_LINK:
      led_period = 600U;
      led_on_time = 300U;
      break;
    case LED_STATUS_CAL_RUNNING:
      led_period = 200U;
      led_on_time = 100U;
      break;
    case LED_STATUS_CAL_OK:
      /* 常亮 2s 后自动切回 NORMAL */
      if (led_mode_elapsed_ms >= 2000U)
      {
        LedStatus_SetMode(LED_STATUS_NORMAL);
      }
      else
      {
        led_period = 2000U;
        led_on_time = 2000U;
      }
      break;
    case LED_STATUS_CAL_APPLIED:
    {
      /* 双闪 3 次: [ON 100, OFF 100, ON 100, OFF 400] × 3 */
      uint32_t cycle = 600U;
      uint32_t phase_ms;

      if (led_mode_elapsed_ms >= 3U * cycle)
      {
        LedStatus_SetMode(LED_STATUS_NORMAL);
      }
      else
      {
        phase_ms = led_mode_elapsed_ms % cycle;
        if (phase_ms < 100U || (phase_ms >= 200U && phase_ms < 300U))
        {
          led_period = 100U;
          led_on_time = 100U;
        }
        else
        {
          led_period = 100U;
          led_on_time = 0U;
        }
      }
      break;
    }
    case LED_STATUS_NORMAL:
    default:
      led_period = 1000U;
      led_on_time = 500U;
      break;
  }
}

void LedStatus_TaskStep(uint32_t period_ms)
{
  uint8_t led_on = 0U;

  led_mode_elapsed_ms += period_ms;
  led_tick_ms += period_ms;

  LedStatus_SelectParams();

  if (led_tick_ms >= led_period)
  {
    led_tick_ms -= led_period;
  }

  if (led_on_time >= led_period)
  {
    led_on = 1U;
  }
  else
  {
    led_on = (led_tick_ms < led_on_time) ? 1U : 0U;
  }

  HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin,
                    (led_on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
