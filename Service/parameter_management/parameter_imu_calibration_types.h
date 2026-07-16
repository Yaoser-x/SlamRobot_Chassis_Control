#ifndef PARAMETER_IMU_CALIBRATION_TYPES_H
#define PARAMETER_IMU_CALIBRATION_TYPES_H

#include <stdint.h>

#define PARAMETER_IMU_CALIBRATION_VERSION 1UL

/** @brief Chip-independent persisted IMU calibration payload. */
typedef struct
{
    uint32_t version;
    float    accel_bias_g[3];
    float    accel_scale[3];
    float    gyro_bias_dps[3];
    float    temperature_offset_c;
    float    temperature_gyro_slope_dps_per_c[3];
    float    sensor_to_body[3][3];
    uint32_t crc;
} imu_calibration_t;

void     ParameterImuCalibration_Default(imu_calibration_t *calibration);
uint32_t ParameterImuCalibration_Crc(const imu_calibration_t *calibration);
uint8_t  ParameterImuCalibration_Validate(const imu_calibration_t *calibration);

#endif /* PARAMETER_IMU_CALIBRATION_TYPES_H */
