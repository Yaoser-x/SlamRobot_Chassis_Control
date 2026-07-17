#ifndef BMI270_TYPES_H
#define BMI270_TYPES_H

#include <stdint.h>

#define BMI270_DRIVER_QUEUE_CAPACITY 8U

typedef enum
{
    BMI270_DRIVER_ERROR_NONE = 0,
    BMI270_DRIVER_ERROR_CHIP_ID,
    BMI270_DRIVER_ERROR_SPI,
    BMI270_DRIVER_ERROR_CONFIG,
    BMI270_DRIVER_ERROR_READ,
    BMI270_DRIVER_ERROR_PROFILE_VERIFY,
    BMI270_DRIVER_ERROR_FIFO,
    BMI270_DRIVER_ERROR_INVALID_FRAME
} bmi270_driver_error_t;

typedef enum
{
    BMI270_DRIVER_INIT_RESET = 0,
    BMI270_DRIVER_INIT_PROBE,
    BMI270_DRIVER_INIT_LOAD_CONFIG,
    BMI270_DRIVER_INIT_VERIFY_PROFILE,
    BMI270_DRIVER_INIT_SAMPLING,
    BMI270_DRIVER_INIT_RETRY_WAIT,
    BMI270_DRIVER_INIT_DISABLED
} bmi270_driver_init_state_t;

/** @brief One BMI270 sample expressed as raw counts and physical sensor units. */
typedef struct
{
    int16_t  accel_raw[3];
    int16_t  gyro_raw[3];
    float    accel_g[3];
    float    gyro_dps[3];
    uint32_t sensor_time;
    uint32_t timestamp_ms;
    uint8_t  sensor_time_valid;
} bmi270_sample_t;

/** @brief BMI270 device, transport, FIFO, and sampling diagnostics only. */
typedef struct
{
    uint8_t  enabled;
    uint8_t  online;
    uint8_t  chip_id;
    uint8_t  last_error;
    uint8_t  init_state;
    uint8_t  profile;
    uint32_t error_count;
    uint32_t last_update_ms;
    uint32_t sample_count;
    uint32_t drdy_count;
    uint32_t poll_fallback_count;
    uint32_t spi_error_count;
    uint32_t init_failure_count;
    uint32_t fifo_error_count;
    float    temperature_c;
    uint8_t  temperature_sampled;
    uint8_t  temperature_valid;
    uint32_t temperature_error_count;
} bmi270_driver_state_t;

#endif /* BMI270_TYPES_H */
