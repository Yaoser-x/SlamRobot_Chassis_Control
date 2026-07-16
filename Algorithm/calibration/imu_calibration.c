#include "imu_calibration.h"

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
        calibration->accel_scale[i]       = 1.0f;
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

    copy     = *calibration;
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

float ImuBmi270Calibration_GyroBiasAtTemperature(const imu_bmi270_calibration_t *calibration,
                                                 uint8_t                         axis,
                                                 float                           temperature_c,
                                                 uint8_t                         temperature_valid)
{
    float bias;

    if (calibration == 0 || axis >= 3U)
    {
        return 0.0f;
    }
    bias = calibration->gyro_bias_dps[axis];
    if (temperature_valid != 0U)
    {
        bias +=
            calibration->temperature_gyro_slope_dps_per_c[axis] * (temperature_c - calibration->temperature_offset_c);
    }
    return bias;
}

void ImuBmi270GyroCalAccumulator_Init(imu_bmi270_gyro_cal_accumulator_t *accumulator)
{
    if (accumulator == 0)
    {
        return;
    }
    memset(accumulator, 0, sizeof(*accumulator));
    accumulator->fail_axis = 0xFFU;
}

uint8_t ImuBmi270GyroCalAccumulator_Begin(imu_bmi270_gyro_cal_accumulator_t *accumulator,
                                          uint16_t                           target_samples,
                                          uint16_t                           interval_ms)
{
    if (accumulator == 0 || target_samples == 0U)
    {
        return 0U;
    }
    ImuBmi270GyroCalAccumulator_Init(accumulator);
    accumulator->target_samples = target_samples;
    accumulator->interval_ms    = interval_ms;
    accumulator->state          = IMU_BMI270_GYRO_CAL_ACC_COLLECTING;
    return 1U;
}

void ImuBmi270GyroCalAccumulator_Restart(imu_bmi270_gyro_cal_accumulator_t *accumulator)
{
    uint16_t target_samples;
    uint16_t interval_ms;

    if (accumulator == 0)
    {
        return;
    }
    target_samples = accumulator->target_samples;
    interval_ms    = accumulator->interval_ms;
    (void)ImuBmi270GyroCalAccumulator_Begin(accumulator, target_samples, interval_ms);
}

imu_bmi270_gyro_cal_acc_state_t ImuBmi270GyroCalAccumulator_Feed(imu_bmi270_gyro_cal_accumulator_t *accumulator,
                                                                 uint32_t                           now_ms,
                                                                 const float                        accel_g[3],
                                                                 const float                        gyro_dps[3],
                                                                 float                              max_abs_dps,
                                                                 float                              max_span_dps)
{
    if (accumulator == 0 || accel_g == 0 || gyro_dps == 0 || accumulator->state != IMU_BMI270_GYRO_CAL_ACC_COLLECTING)
    {
        return (accumulator == 0) ? IMU_BMI270_GYRO_CAL_ACC_IDLE : (imu_bmi270_gyro_cal_acc_state_t)accumulator->state;
    }
    if (accumulator->has_last_sample != 0U && (now_ms - accumulator->last_sample_ms) < accumulator->interval_ms)
    {
        return IMU_BMI270_GYRO_CAL_ACC_COLLECTING;
    }
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        float abs_dps = (gyro_dps[axis] < 0.0f) ? -gyro_dps[axis] : gyro_dps[axis];

        if (abs_dps > max_abs_dps)
        {
            accumulator->state     = IMU_BMI270_GYRO_CAL_ACC_FAIL_ABS;
            accumulator->fail_axis = axis;
            return IMU_BMI270_GYRO_CAL_ACC_FAIL_ABS;
        }
    }
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        if (accumulator->sample_count == 0U)
        {
            accumulator->min_dps[axis] = gyro_dps[axis];
            accumulator->max_dps[axis] = gyro_dps[axis];
        }
        if (gyro_dps[axis] < accumulator->min_dps[axis])
        {
            accumulator->min_dps[axis] = gyro_dps[axis];
        }
        if (gyro_dps[axis] > accumulator->max_dps[axis])
        {
            accumulator->max_dps[axis] = gyro_dps[axis];
        }
        accumulator->sum_dps[axis] += gyro_dps[axis];
        accumulator->accel_sum_g[axis] += accel_g[axis];
    }
    accumulator->sample_count++;
    accumulator->last_sample_ms  = now_ms;
    accumulator->has_last_sample = 1U;
    if (accumulator->sample_count < accumulator->target_samples)
    {
        return IMU_BMI270_GYRO_CAL_ACC_COLLECTING;
    }
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        if ((accumulator->max_dps[axis] - accumulator->min_dps[axis]) > max_span_dps)
        {
            accumulator->state     = IMU_BMI270_GYRO_CAL_ACC_FAIL_SPAN;
            accumulator->fail_axis = axis;
            return IMU_BMI270_GYRO_CAL_ACC_FAIL_SPAN;
        }
    }
    accumulator->state     = IMU_BMI270_GYRO_CAL_ACC_READY;
    accumulator->fail_axis = 0xFFU;
    return IMU_BMI270_GYRO_CAL_ACC_READY;
}

uint8_t ImuBmi270GyroCalAccumulator_GetResult(const imu_bmi270_gyro_cal_accumulator_t *accumulator,
                                              float                                    bias_dps[3],
                                              float                                    accel_mean_g[3])
{
    if (accumulator == 0 || bias_dps == 0 || accel_mean_g == 0 || accumulator->state != IMU_BMI270_GYRO_CAL_ACC_READY
        || accumulator->sample_count == 0U)
    {
        return 0U;
    }
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        bias_dps[axis]     = accumulator->sum_dps[axis] / (float)accumulator->sample_count;
        accel_mean_g[axis] = accumulator->accel_sum_g[axis] / (float)accumulator->sample_count;
    }
    return 1U;
}
