#include "parameter_imu_calibration_types.h"

#include <string.h>

static uint32_t ParameterImuCalibration_Fnv1a(const uint8_t *data, uint32_t length)
{
    uint32_t hash = 2166136261UL;

    for (uint32_t index = 0U; index < length; ++index)
    {
        hash ^= data[index];
        hash *= 16777619UL;
    }
    return hash;
}

void ParameterImuCalibration_Default(imu_calibration_t *calibration)
{
    if (calibration == 0)
    {
        return;
    }
    memset(calibration, 0, sizeof(*calibration));
    calibration->version = PARAMETER_IMU_CALIBRATION_VERSION;
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        calibration->accel_scale[axis]          = 1.0f;
        calibration->sensor_to_body[axis][axis] = 1.0f;
    }
    calibration->crc = ParameterImuCalibration_Crc(calibration);
}

uint32_t ParameterImuCalibration_Crc(const imu_calibration_t *calibration)
{
    imu_calibration_t copy;

    if (calibration == 0)
    {
        return 0UL;
    }
    copy     = *calibration;
    copy.crc = 0UL;
    return ParameterImuCalibration_Fnv1a((const uint8_t *)&copy, sizeof(copy));
}

uint8_t ParameterImuCalibration_Validate(const imu_calibration_t *calibration)
{
    return (calibration != 0 && calibration->version == PARAMETER_IMU_CALIBRATION_VERSION
            && calibration->crc == ParameterImuCalibration_Crc(calibration))
               ? 1U
               : 0U;
}
