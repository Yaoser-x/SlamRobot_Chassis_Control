#ifndef IMU_BMI270_H
#define IMU_BMI270_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  IMU_BMI270_ERROR_NONE = 0,
  IMU_BMI270_ERROR_CHIP_ID = 1,
  IMU_BMI270_ERROR_SPI = 2,
  IMU_BMI270_ERROR_CONFIG = 3,
  IMU_BMI270_ERROR_READ = 4
} imu_bmi270_error_t;

typedef struct
{
  uint8_t enabled;
  uint8_t online;
  uint8_t chip_id;
  uint8_t last_error;
  uint32_t error_count;
  uint32_t last_update_ms;
  int16_t accel_raw[3];
  int16_t gyro_raw[3];
  float accel_g[3];
  float gyro_dps[3];
  float gyro_bias_dps[3];
  float gyro_corrected_dps[3];
  float gyro_filtered_dps[3];
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  float temperature_c;
  uint8_t gyro_calibrated;
  uint8_t filter_initialized;
} imu_bmi270_state_t;

typedef struct
{
  uint8_t hal_status[2];
  uint8_t hal_rx[2][3];
  uint8_t bitbang_rx[3];
  uint8_t miso_nopull;
  uint8_t miso_pullup;
  uint8_t miso_pulldown;
} imu_bmi270_diag_t;

void ImuBmi270_Init(void);
uint8_t ImuBmi270_SetEnabled(uint8_t enabled);
uint8_t ImuBmi270_ProbeNow(void);
uint8_t ImuBmi270_ConfigNow(void);
uint8_t ImuBmi270_Update(void);
uint8_t ImuBmi270_ReadReg(uint8_t reg, uint8_t *value);
uint8_t ImuBmi270_WriteReg(uint8_t reg, uint8_t value);
uint8_t ImuBmi270_Diagnose(imu_bmi270_diag_t *diag);
uint8_t ImuBmi270_CalibrateGyro(uint16_t samples, uint16_t delay_ms);
void ImuBmi270_ClearCalibration(void);
void ImuBmi270_GetState(imu_bmi270_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
