#ifndef ADC_MONITOR_H
#define ADC_MONITOR_H

#include <stdint.h>

#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_MONITOR_VALID_SAMPLES_READY       (1UL << 0)
#define ADC_MONITOR_VALID_CURRENT_ZERO_READY  (1UL << 1)
#define ADC_MONITOR_INVALID_NOT_READY         (1UL << 0)
#define ADC_MONITOR_INVALID_ZERO_CALIBRATING  (1UL << 1)
#define ADC_MONITOR_INVALID_NO_NEW_SAMPLE     (1UL << 2)
#define ADC_MONITOR_INVALID_DMA_ERROR         (1UL << 3)
#define ADC_MONITOR_INVALID_SAMPLE_RATE       (1UL << 4)
#define ADC_MONITOR_INVALID_ZERO_UNSTABLE     (1UL << 5)
#define ADC_MONITOR_INVALID_WINDOW_TOO_SMALL  (1UL << 6)
#define ADC_MONITOR_INVALID_WINDOW_SPIKE      (1UL << 7)

#define ADC_MONITOR_QUALITY_ZERO_UNSTABLE     (1UL << 0)
#define ADC_MONITOR_QUALITY_WINDOW_TOO_SMALL  (1UL << 1)
#define ADC_MONITOR_QUALITY_WINDOW_SPIKE      (1UL << 2)

typedef struct
{
  uint16_t raw_current[MOTOR_ID_COUNT];
  uint16_t current_zero_raw[MOTOR_ID_COUNT];
  uint16_t raw_battery;
  uint16_t raw_left_current;
  uint16_t raw_right_current;
  float current_a[MOTOR_ID_COUNT];
  float current_mean_a[MOTOR_ID_COUNT];
  float current_rms_a[MOTOR_ID_COUNT];
  float current_peak_a[MOTOR_ID_COUNT];
  float current_signed_mean_a[MOTOR_ID_COUNT];
  float current_noise_a[MOTOR_ID_COUNT];
  float battery_voltage;
  float left_current_a;
  float right_current_a;
  uint16_t current_zero_sample_count;
  uint16_t current_sample_count[MOTOR_ID_COUNT];
  uint16_t current_zero_span_raw[MOTOR_ID_COUNT];
  uint16_t raw_sample_count;
  uint16_t missed_window_count;
  uint32_t sample_rate_hz_milli;
  uint32_t current_quality_flags[MOTOR_ID_COUNT];
  uint32_t valid_flags;
  uint32_t invalid_reason_flags;
  uint8_t samples_ready;
  uint8_t current_zero_valid;
  uint8_t current_valid;
  uint8_t current_control_valid;
  uint8_t current_control_valid_mask;
} adc_monitor_state_t;

void AdcMonitor_Init(void);
void AdcMonitor_Update(void);
void AdcMonitor_GetState(adc_monitor_state_t *state);
void AdcMonitor_RequestCurrentZeroCalibration(void);
void AdcMonitor_ApplyCurrentZeroCalibration(const uint16_t zero_raw[MOTOR_ID_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
