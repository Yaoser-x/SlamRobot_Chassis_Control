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

typedef enum
{
  IMU_BMI270_GYRO_CAL_ACC_IDLE = 0,
  IMU_BMI270_GYRO_CAL_ACC_COLLECTING = 1,
  IMU_BMI270_GYRO_CAL_ACC_READY = 2,
  IMU_BMI270_GYRO_CAL_ACC_FAIL_ABS = 3,
  IMU_BMI270_GYRO_CAL_ACC_FAIL_SPAN = 4
} imu_bmi270_gyro_cal_acc_state_t;

typedef struct
{
  float sum_dps[3];
  float accel_sum_g[3];
  float min_dps[3];
  float max_dps[3];
  uint32_t last_sample_ms;
  uint16_t target_samples;
  uint16_t sample_count;
  uint16_t interval_ms;
  uint8_t state;
  uint8_t fail_axis;
  uint8_t has_last_sample;
} imu_bmi270_gyro_cal_accumulator_t;

void ImuBmi270Calibration_Default(imu_bmi270_calibration_t *calibration);
uint32_t ImuBmi270Calibration_Crc(const imu_bmi270_calibration_t *calibration);
uint8_t ImuBmi270Calibration_Validate(const imu_bmi270_calibration_t *calibration);
/** Return the calibrated gyro bias at the measured temperature. */
float ImuBmi270Calibration_GyroBiasAtTemperature(
    const imu_bmi270_calibration_t *calibration,
    uint8_t axis,
    float temperature_c,
    uint8_t temperature_valid);

/** Reset a gyro calibration accumulator to idle. */
void ImuBmi270GyroCalAccumulator_Init(imu_bmi270_gyro_cal_accumulator_t *accumulator);
/** Start collecting a fixed number of gyro samples. */
uint8_t ImuBmi270GyroCalAccumulator_Begin(imu_bmi270_gyro_cal_accumulator_t *accumulator,
                                          uint16_t target_samples,
                                          uint16_t interval_ms);
/** Discard collected samples while preserving the request parameters. */
void ImuBmi270GyroCalAccumulator_Restart(imu_bmi270_gyro_cal_accumulator_t *accumulator);
/** Consume at most one due sample and update validation state. */
imu_bmi270_gyro_cal_acc_state_t ImuBmi270GyroCalAccumulator_Feed(
    imu_bmi270_gyro_cal_accumulator_t *accumulator,
    uint32_t now_ms,
    const float accel_g[3],
    const float gyro_dps[3],
    float max_abs_dps,
    float max_span_dps);
/** Return means only after all samples pass validation. */
uint8_t ImuBmi270GyroCalAccumulator_GetResult(
    const imu_bmi270_gyro_cal_accumulator_t *accumulator,
    float bias_dps[3],
    float accel_mean_g[3]);

#ifdef __cplusplus
}
#endif

#endif
