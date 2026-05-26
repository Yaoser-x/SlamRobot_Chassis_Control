#include "adc_monitor.h"

#include "adc.h"
#include "chassis_config.h"

static uint16_t adc_dma_buffer[ADC_MONITOR_CHANNEL_COUNT];
static volatile uint16_t adc_sample_snapshot[ADC_MONITOR_CHANNEL_COUNT];
static adc_monitor_state_t adc_state;
static uint8_t current_filter_initialized;

static float AdcMonitor_RawToVoltage(uint16_t raw)
{
  return ((float)raw * ADC_MONITOR_VREF_V) / ADC_MONITOR_RESOLUTION_COUNTS;
}

static float AdcMonitor_VoltageToCurrent(float voltage)
{
  float current = 0.0f;

  if (MOTOR_CURRENT_VOLTS_PER_AMP > 0.0f)
  {
    current = (voltage - MOTOR_CURRENT_ZERO_V) / MOTOR_CURRENT_VOLTS_PER_AMP;
    if (current < 0.0f)
    {
      current = -current;
    }
  }
  return current;
}

static float AdcMonitor_FilterCurrent(float previous, float sample)
{
  float alpha = MOTOR_CURRENT_FILTER_ALPHA;

  if (alpha <= 0.0f)
  {
    return previous;
  }
  if (alpha >= 1.0f || current_filter_initialized == 0U)
  {
    return sample;
  }
  return previous + (alpha * (sample - previous));
}

void AdcMonitor_Init(void)
{
  adc_state = (adc_monitor_state_t){0};
  for (uint8_t i = 0U; i < ADC_MONITOR_CHANNEL_COUNT; ++i)
  {
    adc_sample_snapshot[i] = 0U;
  }
  current_filter_initialized = 0U;
  (void)HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_MONITOR_CHANNEL_COUNT);
}

void AdcMonitor_Update(void)
{
  uint16_t raw_current[MOTOR_ID_COUNT];
  uint16_t raw_battery;
  adc_monitor_state_t next_state;
  float previous_current[MOTOR_ID_COUNT];
  uint8_t next_current_valid;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    raw_current[i] = adc_sample_snapshot[i];
    previous_current[i] = adc_state.current_a[i];
  }
  raw_battery = adc_sample_snapshot[4];
  __set_PRIMASK(primask);

  next_state = (adc_monitor_state_t){0};
  next_state.raw_battery = raw_battery;
  next_state.battery_voltage = AdcMonitor_RawToVoltage(raw_battery) * ADC_MONITOR_BATTERY_DIVIDER;
  next_current_valid = (ADC_MONITOR_CALIBRATION_ENABLED != 0U && MOTOR_CURRENT_VOLTS_PER_AMP > 0.0f) ? 1U : 0U;
  next_state.current_valid = next_current_valid;

  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    float current = AdcMonitor_VoltageToCurrent(AdcMonitor_RawToVoltage(raw_current[i]));
    next_state.raw_current[i] = raw_current[i];
    if (next_current_valid != 0U)
    {
      next_state.current_a[i] = AdcMonitor_FilterCurrent(previous_current[i], current);
    }
    else
    {
      next_state.current_a[i] = 0.0f;
    }
  }

  next_state.raw_left_current = (uint16_t)(((uint32_t)next_state.raw_current[MOTOR_ID_M1] +
                                            (uint32_t)next_state.raw_current[MOTOR_ID_M2]) / 2U);
  next_state.raw_right_current = (uint16_t)(((uint32_t)next_state.raw_current[MOTOR_ID_M3] +
                                             (uint32_t)next_state.raw_current[MOTOR_ID_M4]) / 2U);
  next_state.left_current_a = (next_state.current_a[MOTOR_ID_M1] + next_state.current_a[MOTOR_ID_M2]) * 0.5f;
  next_state.right_current_a = (next_state.current_a[MOTOR_ID_M3] + next_state.current_a[MOTOR_ID_M4]) * 0.5f;

  primask = __get_PRIMASK();
  __disable_irq();
  adc_state = next_state;
  current_filter_initialized = next_current_valid;
  __set_PRIMASK(primask);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc != &hadc1)
  {
    return;
  }

  for (uint8_t i = 0U; i < ADC_MONITOR_CHANNEL_COUNT; ++i)
  {
    adc_sample_snapshot[i] = adc_dma_buffer[i];
  }
}

void AdcMonitor_GetState(adc_monitor_state_t *state)
{
  uint32_t primask;

  if (state == 0)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *state = adc_state;
  __set_PRIMASK(primask);
}
