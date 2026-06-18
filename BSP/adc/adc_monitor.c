#include "adc_monitor.h"

#include "adc.h"
#include "chassis_config.h"
#include "chassis_layout.h"

static uint16_t adc_dma_buffer[ADC_MONITOR_CHANNEL_COUNT];
static volatile uint16_t adc_sample_snapshot[ADC_MONITOR_CHANNEL_COUNT];
static volatile uint8_t adc_samples_ready;
static adc_monitor_state_t adc_state;
static uint8_t current_filter_initialized;
static uint8_t battery_filter_initialized;
static uint8_t current_zero_valid;
static uint16_t current_zero_sample_count;
static uint16_t current_zero_raw[MOTOR_ID_COUNT];
static uint32_t current_zero_sum[MOTOR_ID_COUNT];

/* ADC DMA order follows CubeMX ranks; logical M2/M3 are swapped here. */
static const uint8_t current_adc_index[MOTOR_ID_COUNT] = {
  0U,
  2U,
  1U,
  3U,
};

static float AdcMonitor_RawToVoltage(uint16_t raw)
{
  return ((float)raw * ADC_MONITOR_VREF_V) / ADC_MONITOR_RESOLUTION_COUNTS;
}

static float AdcMonitor_RawDeltaToCurrent(uint16_t raw, uint16_t zero_raw)
{
  float current = 0.0f;

  if (MOTOR_CURRENT_VOLTS_PER_AMP > 0.0f)
  {
    uint16_t raw_delta = (raw >= zero_raw) ? (uint16_t)(raw - zero_raw) : (uint16_t)(zero_raw - raw);
    current = AdcMonitor_RawToVoltage(raw_delta) / MOTOR_CURRENT_VOLTS_PER_AMP;
  }
  return current;
}

static float AdcMonitor_FilterBattery(float previous, float sample)
{
  float alpha = ADC_MONITOR_BATTERY_FILTER_ALPHA;

  if (alpha <= 0.0f)
  {
    return previous;
  }
  if (alpha >= 1.0f || battery_filter_initialized == 0U)
  {
    return sample;
  }
  return previous + (alpha * (sample - previous));
}

static void AdcMonitor_UpdateCurrentZero(const uint16_t raw_current[MOTOR_ID_COUNT])
{
  if (ADC_MONITOR_CALIBRATION_ENABLED == 0U || current_zero_valid != 0U)
  {
    return;
  }

  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    current_zero_sum[i] += raw_current[i];
  }
  current_zero_sample_count++;

  if (current_zero_sample_count >= ADC_MONITOR_CURRENT_ZERO_SAMPLES)
  {
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
      current_zero_raw[i] = (uint16_t)(current_zero_sum[i] / (uint32_t)current_zero_sample_count);
    }
    current_zero_valid = 1U;
  }
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
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    current_zero_raw[i] = 0U;
    current_zero_sum[i] = 0U;
  }
  adc_samples_ready = 0U;
  current_filter_initialized = 0U;
  battery_filter_initialized = 0U;
  current_zero_valid = 0U;
  current_zero_sample_count = 0U;
  (void)HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_MONITOR_CHANNEL_COUNT);
}

void AdcMonitor_Update(void)
{
  uint16_t raw_current[MOTOR_ID_COUNT];
  uint16_t zero_raw[MOTOR_ID_COUNT];
  uint16_t raw_battery;
  adc_monitor_state_t next_state;
  float previous_current[MOTOR_ID_COUNT];
  float previous_battery;
  uint8_t samples_ready;
  uint8_t zero_valid;
  uint16_t zero_sample_count;
  uint8_t next_current_valid;
  uint32_t primask;
  uint8_t left_count = 0U;
  uint8_t right_count = 0U;
  uint32_t left_raw_sum = 0U;
  uint32_t right_raw_sum = 0U;
  float left_current_sum = 0.0f;
  float right_current_sum = 0.0f;

  primask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    raw_current[i] = adc_sample_snapshot[current_adc_index[i]];
    zero_raw[i] = current_zero_raw[i];
    previous_current[i] = adc_state.current_a[i];
  }
  raw_battery = adc_sample_snapshot[4];
  previous_battery = adc_state.battery_voltage;
  samples_ready = adc_samples_ready;
  zero_valid = current_zero_valid;
  zero_sample_count = current_zero_sample_count;
  __set_PRIMASK(primask);

  if (samples_ready != 0U)
  {
    AdcMonitor_UpdateCurrentZero(raw_current);
  }

  primask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    zero_raw[i] = current_zero_raw[i];
  }
  zero_valid = current_zero_valid;
  zero_sample_count = current_zero_sample_count;
  __set_PRIMASK(primask);

  next_state = (adc_monitor_state_t){0};
  next_state.raw_battery = raw_battery;
  next_state.battery_voltage = AdcMonitor_FilterBattery(previous_battery,
                                                       AdcMonitor_RawToVoltage(raw_battery) *
                                                       ADC_MONITOR_BATTERY_DIVIDER);
  next_state.samples_ready = samples_ready;
  next_state.current_zero_valid = zero_valid;
  next_state.current_zero_sample_count = zero_sample_count;
  next_current_valid = (samples_ready != 0U &&
                        zero_valid != 0U &&
                        ADC_MONITOR_CALIBRATION_ENABLED != 0U &&
                        MOTOR_CURRENT_VOLTS_PER_AMP > 0.0f) ? 1U : 0U;
  next_state.current_valid = next_current_valid;

  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    float current = AdcMonitor_RawDeltaToCurrent(raw_current[i], zero_raw[i]);
    next_state.raw_current[i] = raw_current[i];
    next_state.current_zero_raw[i] = zero_raw[i];
    if (next_current_valid != 0U)
    {
      next_state.current_a[i] = AdcMonitor_FilterCurrent(previous_current[i], current);
    }
    else
    {
      next_state.current_a[i] = 0.0f;
    }

    if (ChassisLayout_MotorEnabled((motor_id_t)i) == 0U)
    {
      next_state.raw_current[i] = 0U;
      next_state.current_a[i] = 0.0f;
    }
    else if (ChassisLayout_MotorSide((motor_id_t)i) == MOTOR_SIDE_LEFT)
    {
      left_count++;
      left_raw_sum += next_state.raw_current[i];
      left_current_sum += next_state.current_a[i];
    }
    else
    {
      right_count++;
      right_raw_sum += next_state.raw_current[i];
      right_current_sum += next_state.current_a[i];
    }
  }

  next_state.raw_left_current = (left_count != 0U) ? (uint16_t)(left_raw_sum / left_count) : 0U;
  next_state.raw_right_current = (right_count != 0U) ? (uint16_t)(right_raw_sum / right_count) : 0U;
  next_state.left_current_a = (left_count != 0U) ? (left_current_sum / (float)left_count) : 0.0f;
  next_state.right_current_a = (right_count != 0U) ? (right_current_sum / (float)right_count) : 0.0f;

  primask = __get_PRIMASK();
  __disable_irq();
  adc_state = next_state;
  current_filter_initialized = next_current_valid;
  battery_filter_initialized = samples_ready;
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
  adc_samples_ready = 1U;
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
