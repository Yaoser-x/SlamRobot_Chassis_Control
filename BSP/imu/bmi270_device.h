#ifndef BMI270_DEVICE_H
#define BMI270_DEVICE_H

#include <stdint.h>

#include "bmi270_profile.h"

typedef struct
{
    uint8_t (*read_reg)(uint8_t reg, uint8_t *value);
    uint8_t (*write_reg)(uint8_t reg, uint8_t value);
    uint8_t (*write_bytes)(uint8_t reg, const uint8_t *data, uint8_t len);
} bmi270_device_io_t;

typedef enum
{
    BMI270_DEVICE_OK = 0,
    BMI270_DEVICE_IO_ERROR,
    BMI270_DEVICE_CONFIG_ERROR,
    BMI270_DEVICE_PROFILE_MISMATCH
} bmi270_device_status_t;

/** Upload the fixed BMI270 configuration image. */
bmi270_device_status_t Bmi270Device_LoadConfig(const bmi270_device_io_t *io);

/** Wait until the BMI270 internal initialization status reports success. */
bmi270_device_status_t Bmi270Device_WaitInitOk(const bmi270_device_io_t *io);

/** Apply and verify one immutable BMI270 runtime profile. */
bmi270_device_status_t Bmi270Device_ApplyProfile(const bmi270_device_io_t *io, const imu_bmi270_profile_t *profile);

#endif
