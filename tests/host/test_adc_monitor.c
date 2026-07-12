#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "adc_monitor.h"
#include "adc.h"
#include "chassis_config.h"
#include "tim.h"

ADC_HandleTypeDef  hadc1;
uint32_t           host_dma_disabled_interrupt_mask;
static TIM_TypeDef tim8_instance = {0};
TIM_HandleTypeDef  htim8         = {.Instance = &tim8_instance};

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

static uint32_t  fake_primask;
static uint16_t *fake_adc_dma;
static uint32_t  fake_adc_dma_len;
static uint32_t  fake_adc_start_count;
static uint32_t  fake_tim8_base_start_count;
static uint32_t  fake_tick;

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
    fake_adc_dma     = (uint16_t *)buffer;
    fake_adc_dma_len = length;
    fake_adc_start_count++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim)
{
    if (htim == &htim8)
    {
        fake_tim8_base_start_count++;
    }
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
    return ((float)raw * ADC_MONITOR_VREF_V * ADC_MONITOR_BATTERY_DIVIDER) / ADC_MONITOR_RESOLUTION_COUNTS;
}

static float raw_delta_to_current(uint16_t raw_delta)
{
    return ((float)raw_delta * ADC_MONITOR_VREF_V) / (ADC_MONITOR_RESOLUTION_COUNTS * MOTOR_CURRENT_VOLTS_PER_AMP);
}

static float raw_delta_to_m2_current(uint16_t raw_delta)
{
    return ((float)raw_delta * ADC_MONITOR_VREF_V) / (ADC_MONITOR_RESOLUTION_COUNTS * MOTOR_CURRENT_VOLTS_PER_AMP_M2);
}

static void test_current_zero_requires_startup_samples(void)
{
    adc_monitor_state_t state            = {0};
    uint32_t            start_count      = fake_adc_start_count;
    uint32_t            tim8_start_count = fake_tim8_base_start_count;

    AdcMonitor_Init();
    AdcMonitor_SetCurrentZeroStationary(1U);
    require_int(fake_adc_start_count == start_count + 1U, "adc dma started once");
    require_int(fake_tim8_base_start_count == tim8_start_count + 1U, "tim8 base started once");
    require_int((host_dma_disabled_interrupt_mask & DMA_IT_HT) != 0U, "adc disables half-transfer irq");
    require_int((host_dma_disabled_interrupt_mask & DMA_IT_TC) == 0U, "adc keeps transfer-complete irq");
    require_int(fake_adc_dma_len == ADC_MONITOR_CHANNEL_COUNT, "adc dma length");
    require_close(raw_delta_to_current(100U), 0.0806f, 0.0002f, "current scale is 1.0 V/A");

    for (uint16_t i = 0U; i < (uint16_t)(ADC_MONITOR_CURRENT_ZERO_SAMPLES - 1U); ++i)
    {
        push_adc_sample(100U, 110U, 120U, 130U, 2700U);
    }
    AdcMonitor_Update();
    AdcMonitor_GetState(&state);
    require_int(state.samples_ready != 0U, "adc samples ready");
    require_int(state.current_zero_valid == 0U, "zero not ready before sample target");
    require_int(state.current_valid == 0U, "current invalid before zero");
    require_int(state.current_control_valid == 0U, "current control invalid before zero");

    push_adc_sample(100U, 110U, 120U, 130U, 2700U);
    AdcMonitor_Update();
    AdcMonitor_GetState(&state);
    require_int(state.current_zero_valid != 0U, "zero ready at sample target");
    require_int(state.current_valid == 0U, "current invalid until post-zero delta window");
    require_int(state.current_control_valid == 0U, "current control invalid until post-zero delta window");
    require_int(state.current_zero_raw[MOTOR_ID_M1] == 100U, "m1 zero raw");
    require_int(state.current_zero_raw[MOTOR_ID_M2] == 110U, "m2 zero raw");
    require_int(state.current_zero_raw[MOTOR_ID_M3] == 120U, "m3 zero raw");
    require_int(state.current_zero_raw[MOTOR_ID_M4] == 130U, "m4 zero raw");
    require_close(state.current_a[MOTOR_ID_M2], 0.0f, 0.0001f, "m2 zero current");

    push_adc_sample(100U, 110U, 120U, 130U, 2700U);
    AdcMonitor_Update();
    AdcMonitor_GetState(&state);
    require_int(state.current_valid != 0U, "current valid after post-zero sample");
    require_int(state.current_control_valid == 0U, "single-sample current is not control-valid");
    require_int((state.invalid_reason_flags & ADC_MONITOR_INVALID_WINDOW_TOO_SMALL) != 0U,
                "single-sample current reports small window");

    for (uint8_t i = 0U; i < 19U; ++i)
    {
        push_adc_sample(100U, 100U, 130U, 130U, 2700U);
    }
    push_adc_sample(100U, 100U, 220U, 130U, 2700U);
    AdcMonitor_Update();
    AdcMonitor_GetState(&state);
    require_close(state.current_a[MOTOR_ID_M2], 0.0f, 0.0001f, "m2 negative drift remains zero");
    require_close(state.current_mean_a[MOTOR_ID_M2], 0.0f, 0.0001f, "m2 negative drift window mean zero");
    require_close(state.current_rms_a[MOTOR_ID_M2], 0.0f, 0.0001f, "m2 negative drift window rms zero");
    require_close(state.current_peak_a[MOTOR_ID_M2], 0.0f, 0.0001f, "m2 negative drift peak zero");
    require_close(state.current_signed_mean_a[MOTOR_ID_M2],
                  -raw_delta_to_m2_current(10U),
                  0.001f,
                  "m2 signed mean preserves negative drift");
    require_int(state.current_control_valid != 0U, "20-sample window is control-valid");
    require_int(state.current_sample_count[MOTOR_ID_M3] == 20U, "m3 window sample count");
    require_close(state.current_mean_a[MOTOR_ID_M3],
                  raw_delta_to_current(14U) + (raw_delta_to_current(1U) * 0.5f),
                  0.001f,
                  "m3 window mean current");
    require_close(state.current_rms_a[MOTOR_ID_M3],
                  raw_delta_to_current(24U) + (raw_delta_to_current(1U) * 0.3927f),
                  0.001f,
                  "m3 window rms current");
    require_close(state.current_peak_a[MOTOR_ID_M3], raw_delta_to_current(100U), 0.001f, "m3 window peak current");
    require_close(state.current_a[MOTOR_ID_M3],
                  raw_delta_to_current(10U) * MOTOR_CURRENT_FILTER_ALPHA,
                  0.001f,
                  "m3 slow current uses trimmed sample");
}

static void test_current_rezero_requires_fresh_stable_window(void)
{
    adc_monitor_state_t state = {0};

    AdcMonitor_Init();
    AdcMonitor_SetCurrentZeroStationary(1U);
    for (uint16_t i = 0U; i < ADC_MONITOR_CURRENT_ZERO_SAMPLES; ++i)
    {
        push_adc_sample(100U, 110U, 120U, 130U, 2700U);
    }
    AdcMonitor_Update();
    for (uint8_t i = 0U; i < 20U; ++i)
    {
        push_adc_sample(100U, 110U, 120U, 130U, 2700U);
    }
    AdcMonitor_Update();
    AdcMonitor_GetState(&state);
    require_int(state.current_control_valid != 0U, "initial zero becomes control-valid");

    AdcMonitor_RequestCurrentZeroCalibration();
    AdcMonitor_SetCurrentZeroStationary(1U);
    AdcMonitor_GetState(&state);
    require_int(state.current_zero_valid == 0U, "rezero clears zero-valid");
    require_int(state.current_control_valid == 0U, "rezero clears control-valid");

    for (uint16_t i = 0U; i < ADC_MONITOR_CURRENT_ZERO_SAMPLES; ++i)
    {
        uint16_t jitter = (uint16_t)((i & 1U) ? 40U : 0U);
        push_adc_sample(200U, (uint16_t)(210U + jitter), 220U, 230U, 2700U);
    }
    AdcMonitor_Update();
    for (uint8_t i = 0U; i < 20U; ++i)
    {
        push_adc_sample(200U, 210U, 220U, 230U, 2700U);
    }
    AdcMonitor_Update();
    AdcMonitor_GetState(&state);

    require_int(state.current_zero_valid != 0U, "unstable rezero still completes display zero");
    require_int(state.current_control_valid == 0U, "unstable rezero blocks control-valid");
    require_int(state.current_zero_span_raw[MOTOR_ID_M2] == 40U, "zero span exports raw jitter");
    require_int((state.current_quality_flags[MOTOR_ID_M2] & ADC_MONITOR_QUALITY_ZERO_UNSTABLE) != 0U,
                "unstable zero quality flag set");
    require_int((state.invalid_reason_flags & ADC_MONITOR_INVALID_ZERO_UNSTABLE) != 0U,
                "unstable zero invalid reason set");
}

static void test_current_validity_tracks_missing_windows(void)
{
    adc_monitor_state_t state = {0};

    AdcMonitor_Init();
    AdcMonitor_SetCurrentZeroStationary(1U);
    for (uint16_t i = 0U; i < ADC_MONITOR_CURRENT_ZERO_SAMPLES; ++i)
    {
        push_adc_sample(100U, 110U, 120U, 130U, 2700U);
    }
    AdcMonitor_Update();
    push_adc_sample(100U, 110U, 120U, 130U, 2700U);
    AdcMonitor_Update();
    AdcMonitor_GetState(&state);
    require_int(state.current_valid != 0U, "current valid after calibrated sample window");
    require_int(state.current_control_valid == 0U, "one-sample calibrated window is not control-valid");
    require_int(state.raw_sample_count == 1U, "raw sample count exported");
    require_int((state.invalid_reason_flags & ADC_MONITOR_INVALID_WINDOW_TOO_SMALL) != 0U,
                "small valid window has invalid reason for control");

    fake_tick += 20U;
    AdcMonitor_Update();
    AdcMonitor_GetState(&state);
    require_int(state.current_valid == 0U, "current invalid without new samples");
    require_int((state.invalid_reason_flags & ADC_MONITOR_INVALID_NO_NEW_SAMPLE) != 0U, "no new sample reason set");
    require_int(state.missed_window_count == 1U, "missed window count increments");
}

static void test_current_zero_motion_discards_partial_window(void)
{
    adc_monitor_state_t state = {0};

    AdcMonitor_Init();
    AdcMonitor_SetCurrentZeroStationary(1U);
    for (uint16_t i = 0U; i < 64U; ++i)
    {
        push_adc_sample(100U, 110U, 120U, 130U, 2700U);
    }
    AdcMonitor_Update();
    AdcMonitor_GetState(&state);
    require_int(state.current_zero_sample_count == 64U, "partial zero samples accumulate while stationary");

    AdcMonitor_SetCurrentZeroStationary(0U);
    AdcMonitor_GetState(&state);
    require_int(state.current_zero_sample_count == 0U, "motion clears partial zero samples");
    require_int(state.current_zero_valid == 0U, "motion keeps zero invalid");

    AdcMonitor_SetCurrentZeroStationary(1U);
    for (uint16_t i = 0U; i < ADC_MONITOR_CURRENT_ZERO_SAMPLES - 1U; ++i)
    {
        push_adc_sample(200U, 210U, 220U, 230U, 2700U);
    }
    AdcMonitor_Update();
    AdcMonitor_GetState(&state);
    require_int(state.current_zero_valid == 0U, "restart requires a complete fresh zero window");
    push_adc_sample(200U, 210U, 220U, 230U, 2700U);
    AdcMonitor_Update();
    AdcMonitor_GetState(&state);
    require_int(state.current_zero_valid != 0U, "fresh stationary window completes zero");
    require_int(state.current_zero_raw[MOTOR_ID_M1] == 200U, "discarded samples do not pollute new zero");
}

static void test_battery_voltage_is_filtered(void)
{
    adc_monitor_state_t state = {0};
    float               first_voltage;
    float               expected_step;
    uint32_t            start_count = fake_adc_start_count;

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
    expected_step = first_voltage + (ADC_MONITOR_BATTERY_FILTER_ALPHA * (raw_to_vbat(2800U) - first_voltage));
    require_close(state.battery_voltage, expected_step, 0.001f, "battery ema step");
    require_int(state.raw_battery == 2800U, "raw battery keeps latest sample");
}

int main(void)
{
    test_current_zero_requires_startup_samples();
    test_current_rezero_requires_fresh_stable_window();
    test_current_validity_tracks_missing_windows();
    test_current_zero_motion_discards_partial_window();
    test_battery_voltage_is_filtered();
    (void)printf("PASS: adc monitor host tests\n");
    return 0;
}
