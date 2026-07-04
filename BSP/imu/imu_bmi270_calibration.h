#ifndef IMU_BMI270_CALIBRATION_H
#define IMU_BMI270_CALIBRATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_BMI270_CALIBRATION_VERSION 1UL

typedef struct
{
  uint32_t version;
  float accel_bias_g[3];
  float accel_scale[3];
  float gyro_bias_dps[3];
  float temperature_offset_c;
  float temperature_gyro_slope_dps_per_c[3];
  float sensor_to_body[3][3];
  uint32_t crc;
} imu_bmi270_calibration_t;

void ImuBmi270Calibration_Default(imu_bmi270_calibration_t *calibration);
uint32_t ImuBmi270Calibration_Crc(const imu_bmi270_calibration_t *calibration);
uint8_t ImuBmi270Calibration_Validate(const imu_bmi270_calibration_t *calibration);
uint8_t ImuBmi270Calibration_Load(imu_bmi270_calibration_t *calibration);
uint8_t ImuBmi270Calibration_Save(const imu_bmi270_calibration_t *calibration);

#ifdef __cplusplus
}
#endif

#endif
