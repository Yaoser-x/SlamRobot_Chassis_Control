#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "adc_monitor.h"
#include "adc.h"
#include "chassis_config.h"

ADC_HandleTypeDef hadc1;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

static uint32_t fake_primask;
static uint16_t *fake_adc_dma;
static uint32_t fake_adc_dma_len;
static uint32_t fake_adc_start_count;
static uint32_t fake_tick;

uint32_t osKernelGetTickCount(void)
{
  return fake_tick;
}

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

HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *buffer, uint32_t length)
{
  (void)hadc;
  fake_adc_dma = (uint16_t *)buffer;
  fake_adc_dma_len = length;
  fake_adc_start_count++;
  return HAL_OK;
}

static void require_int(int condition, const char *message)
{
  if (condition == 0)
  {
    (void)printf("FAIL: %s\n", message);
    _Exit(1);
  }
}

static void require_close(float actual, float expected, float epsilon, const char *message)
{
  float error = actual - expected;
  if (error < 0.0f)
  {
    error = -error;
  }
  if (error > epsilon)
  {
    (void)printf("FAIL: %s actual=%f expected=%f\n", message, actual, expected);
    _Exit(1);
  }
}

static void push_adc_sample(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4, uint16_t battery)
{
  require_int(fake_adc_dma != 0, "dma buffer captured");
  require_int(fake_adc_dma_len == ADC_MONITOR_CHANNEL_COUNT, "dma channel count");

  fake_adc_dma[0] = m1;
  fake_adc_dma[1] = m2;
  fake_adc_dma[2] = m3;
  fake_adc_dma[3] = m4;
  fake_adc_dma[4] = battery;
  HAL_ADC_ConvCpltCallback(&hadc1);
}

static float raw_to_vbat(uint16_t raw)
{
  return ((float)raw * ADC_MONITOR_VREF_V * ADC_MONITOR_BATTERY_DIVIDER) /
         ADC_MONITOR_RESOLUTION_COUNTS;
}

static void test_current_zero_requires_startup_samples(void)
{
  adc_monitor_state_t state = {0};
  uint32_t start_count = fake_adc_start_count;

  AdcMonitor_Init();
  require_int(fake_adc_start_count == start_count + 1U, "adc dma started once");
  require_int(fake_adc_dma_len == ADC_MONITOR_CHANNEL_COUNT, "adc dma length");
  push_adc_sample(100U, 110U, 120U, 130U, 2700U);

  for (uint16_t i = 0U; i < (uint16_t)(ADC_MONITOR_CURRENT_ZERO_SAMPLES - 1U); ++i)
  {
    AdcMonitor_Update();
  }
  AdcMonitor_GetState(&state);
  require_int(state.samples_ready != 0U, "adc samples ready");
  require_int(state.current_zero_valid == 0U, "zero not ready before sample target");
  require_int(state.current_valid == 0U, "current invalid before zero");

  AdcMonitor_Update();
  AdcMonitor_GetState(&state);
  require_int(state.current_zero_valid != 0U, "zero ready at sample target");
  require_int(state.current_valid != 0U, "current valid after zero");
  require_int(state.current_zero_raw[MOTOR_ID_M1] == 100U, "m1 zero raw");
  require_int(state.current_zero_raw[MOTOR_ID_M2] == 120U, "m2 zero raw");
  require_int(state.current_zero_raw[MOTOR_ID_M3] == 110U, "m3 zero raw");
  require_int(state.current_zero_raw[MOTOR_ID_M4] == 130U, "m4 zero raw");
  require_close(state.current_a[MOTOR_ID_M2], 0.0f, 0.0001f, "m2 zero current");

  push_adc_sample(110U, 110U, 130U, 130U, 2700U);
  AdcMonitor_Update();
  AdcMonitor_GetState(&state);
  require_close(state.current_a[MOTOR_ID_M2], 0.0201f, 0.001f, "m2 filtered current step");
  require_close(state.current_a[MOTOR_ID_M3], 0.0f, 0.0001f, "m3 remains zero");
}

static void test_battery_voltage_is_filtered(void)
{
  adc_monitor_state_t state = {0};
  float first_voltage;
  float expected_step;
  uint32_t start_count = fake_adc_start_count;

  AdcMonitor_Init();
  require_int(fake_adc_start_count == start_count + 1U, "adc dma restarted once");
  push_adc_sample(0U, 0U, 0U, 0U, 2700U);
  AdcMonitor_Update();
  AdcMonitor_GetState(&state);
  first_voltage = raw_to_vbat(2700U);
  require_close(state.battery_voltage, first_voltage, 0.001f, "first battery sample direct");
  require_int(state.raw_battery == 2700U, "raw battery first sample");

  push_adc_sample(0U, 0U, 0U, 0U, 2800U);
  AdcMonitor_Update();
  AdcMonitor_GetState(&state);
  expected_step = first_voltage + (ADC_MONITOR_BATTERY_FILTER_ALPHA *
                                  (raw_to_vbat(2800U) - first_voltage));
  require_close(state.battery_voltage, expected_step, 0.001f, "battery ema step");
  require_int(state.raw_battery == 2800U, "raw battery keeps latest sample");
}

int main(void)
{
  test_current_zero_requires_startup_samples();
  test_battery_voltage_is_filtered();
  (void)printf("PASS: adc monitor host tests\n");
  return 0;
}
