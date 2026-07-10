#ifndef IMU_CALIBRATION_GATE_H
#define IMU_CALIBRATION_GATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_CALIBRATION_GATE_MOTOR_COUNT 4U
#define IMU_CALIBRATION_GATE_WINDOW_SAMPLES 100U

typedef struct
{
  float accel_norm_window[IMU_CALIBRATION_GATE_WINDOW_SAMPLES];
  float gyro_window[3][IMU_CALIBRATION_GATE_WINDOW_SAMPLES];
  float accel_sum;
  float accel_sum_sq;
  float gyro_sum[3];
  float gyro_sum_sq[3];
  uint32_t last_sample_count;
  uint16_t sample_count;
  uint16_t write_index;
  uint8_t has_last_sample;
} imu_calibration_gate_t;

/** Clear the stationary evidence window. */
void ImuCalibrationGate_Init(imu_calibration_gate_t *gate);
/** Evaluate motor, encoder, acceleration, and variance stationary evidence. */
uint8_t ImuCalibrationGate_Update(
    imu_calibration_gate_t *gate,
    const int16_t effective_pwm[IMU_CALIBRATION_GATE_MOTOR_COUNT],
    const float speed_mps[IMU_CALIBRATION_GATE_MOTOR_COUNT],
    const uint8_t speed_valid[IMU_CALIBRATION_GATE_MOTOR_COUNT],
    uint8_t motor_enabled_mask,
    const float accel_g[3],
    const float gyro_dps[3],
    uint32_t imu_sample_count);

#ifdef __cplusplus
}
#endif

#endif
