#ifndef POWER_MANAGEMENT_STATUS_H
#define POWER_MANAGEMENT_STATUS_H

#include <stdint.h>

#define POWER_MANAGEMENT_MOTOR_COUNT                  4U

#define POWER_MANAGEMENT_ADC_VALID_SAMPLES_READY      (1UL << 0)
#define POWER_MANAGEMENT_ADC_VALID_CURRENT_ZERO_READY (1UL << 1)
#define POWER_MANAGEMENT_ADC_INVALID_NOT_READY        (1UL << 0)
#define POWER_MANAGEMENT_ADC_INVALID_ZERO_CALIBRATING (1UL << 1)
#define POWER_MANAGEMENT_ADC_INVALID_NO_NEW_SAMPLE    (1UL << 2)
#define POWER_MANAGEMENT_ADC_INVALID_DMA_ERROR        (1UL << 3)
#define POWER_MANAGEMENT_ADC_INVALID_SAMPLE_RATE      (1UL << 4)
#define POWER_MANAGEMENT_ADC_INVALID_ZERO_UNSTABLE    (1UL << 5)
#define POWER_MANAGEMENT_ADC_INVALID_WINDOW_TOO_SMALL (1UL << 6)
#define POWER_MANAGEMENT_ADC_INVALID_WINDOW_SPIKE     (1UL << 7)

typedef struct
{
    uint16_t raw_current[POWER_MANAGEMENT_MOTOR_COUNT];
    uint16_t current_zero_raw[POWER_MANAGEMENT_MOTOR_COUNT];
    uint16_t raw_battery;
    uint16_t raw_left_current;
    uint16_t raw_right_current;
    float    current_a[POWER_MANAGEMENT_MOTOR_COUNT];
    float    current_mean_a[POWER_MANAGEMENT_MOTOR_COUNT];
    float    current_rms_a[POWER_MANAGEMENT_MOTOR_COUNT];
    float    current_peak_a[POWER_MANAGEMENT_MOTOR_COUNT];
    float    current_signed_mean_a[POWER_MANAGEMENT_MOTOR_COUNT];
    float    current_noise_a[POWER_MANAGEMENT_MOTOR_COUNT];
    float    battery_voltage;
    float    left_current_a;
    float    right_current_a;
    uint16_t current_zero_sample_count;
    uint16_t current_sample_count[POWER_MANAGEMENT_MOTOR_COUNT];
    uint16_t current_zero_span_raw[POWER_MANAGEMENT_MOTOR_COUNT];
    uint16_t raw_sample_count;
    uint16_t missed_window_count;
    uint32_t sample_rate_hz_milli;
    uint32_t current_quality_flags[POWER_MANAGEMENT_MOTOR_COUNT];
    uint32_t valid_flags;
    uint32_t invalid_reason_flags;
    uint8_t  samples_ready;
    uint8_t  current_zero_valid;
    uint8_t  current_valid;
    uint8_t  current_control_valid;
    uint8_t  current_control_valid_mask;
} power_management_status_t;

#endif /* POWER_MANAGEMENT_STATUS_H */
