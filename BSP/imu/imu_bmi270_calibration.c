#include "imu_bmi270_calibration.h"

#include <stddef.h>
#include <string.h>

static uint32_t ImuBmi270Calibration_Fnv1a(const uint8_t *data, uint32_t len)
{
  uint32_t hash = 2166136261UL;

  for (uint32_t i = 0UL; i < len; ++i)
  {
    hash ^= data[i];
    hash *= 16777619UL;
  }
  return hash;
}

void ImuBmi270Calibration_Default(imu_bmi270_calibration_t *calibration)
{
  if (calibration == 0)
  {
    return;
  }

  memset(calibration, 0, sizeof(*calibration));
  calibration->version = IMU_BMI270_CALIBRATION_VERSION;
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    calibration->accel_scale[i] = 1.0f;
    calibration->sensor_to_body[i][i] = 1.0f;
  }
  calibration->crc = ImuBmi270Calibration_Crc(calibration);
}

uint32_t ImuBmi270Calibration_Crc(const imu_bmi270_calibration_t *calibration)
{
  imu_bmi270_calibration_t copy;

  if (calibration == 0)
  {
    return 0UL;
  }

  copy = *calibration;
  copy.crc = 0UL;
  return ImuBmi270Calibration_Fnv1a((const uint8_t *)&copy, (uint32_t)sizeof(copy));
}

uint8_t ImuBmi270Calibration_Validate(const imu_bmi270_calibration_t *calibration)
{
  if (calibration == 0 || calibration->version != IMU_BMI270_CALIBRATION_VERSION)
  {
    return 0U;
  }
  return (calibration->crc == ImuBmi270Calibration_Crc(calibration)) ? 1U : 0U;
}

uint8_t ImuBmi270Calibration_Load(imu_bmi270_calibration_t *calibration)
{
  ImuBmi270Calibration_Default(calibration);
  return 1U;
}

uint8_t ImuBmi270Calibration_Save(const imu_bmi270_calibration_t *calibration)
{
  return ImuBmi270Calibration_Validate(calibration);
}
