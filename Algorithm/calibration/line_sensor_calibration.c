#include "line_sensor_calibration.h"

#include <string.h>

void LineSensorCalibration_Init(line_sensor_calibration_t *calibration)
{
    if (calibration != 0)
    {
        memset(calibration, 0, sizeof(*calibration));
    }
}

uint8_t LineSensorCalibration_Begin(line_sensor_calibration_t        *calibration,
                                    line_sensor_calibration_surface_t surface,
                                    uint16_t                          samples)
{
    uint8_t index = (uint8_t)surface;

    if (calibration == 0 || index > 1U || samples < 4U || samples > 2000U)
    {
        return 0U;
    }
    calibration->count[index]   = 0U;
    calibration->target_samples = samples;
    calibration->surface        = index;
    calibration->collecting     = 1U;
    calibration->ready_mask &= (uint8_t) ~(1U << index);
    for (uint8_t channel = 0U; channel < LINE_CALIBRATION_CHANNELS; ++channel)
    {
        calibration->sum[index][channel] = 0UL;
        calibration->min[index][channel] = UINT16_MAX;
        calibration->max[index][channel] = 0U;
    }
    return 1U;
}

void LineSensorCalibration_Feed(line_sensor_calibration_t *calibration, const uint16_t raw[LINE_CALIBRATION_CHANNELS])
{
    uint8_t index;

    if (calibration == 0 || raw == 0 || calibration->collecting == 0U)
    {
        return;
    }
    index = calibration->surface;
    for (uint8_t channel = 0U; channel < LINE_CALIBRATION_CHANNELS; ++channel)
    {
        calibration->sum[index][channel] += raw[channel];
        if (raw[channel] < calibration->min[index][channel])
        {
            calibration->min[index][channel] = raw[channel];
        }
        if (raw[channel] > calibration->max[index][channel])
        {
            calibration->max[index][channel] = raw[channel];
        }
    }
    calibration->count[index]++;
    if (calibration->count[index] >= calibration->target_samples)
    {
        calibration->collecting = 0U;
        calibration->ready_mask |= (uint8_t)(1U << index);
    }
}

uint8_t LineSensorCalibration_Apply(line_sensor_calibration_t *calibration,
                                    uint16_t                   thresholds[LINE_CALIBRATION_CHANNELS],
                                    uint8_t                   *active_low)
{
    uint32_t floor_total = 0UL;
    uint32_t line_total  = 0UL;

    if (calibration == 0 || thresholds == 0 || active_low == 0 || calibration->ready_mask != 0x03U)
    {
        return 0U;
    }
    calibration->fail_mask = 0U;
    for (uint8_t channel = 0U; channel < LINE_CALIBRATION_CHANNELS; ++channel)
    {
        uint16_t floor_mean = (uint16_t)(calibration->sum[0][channel] / calibration->count[0]);
        uint16_t line_mean  = (uint16_t)(calibration->sum[1][channel] / calibration->count[1]);
        uint16_t separation =
            (floor_mean > line_mean) ? (uint16_t)(floor_mean - line_mean) : (uint16_t)(line_mean - floor_mean);

        if (separation < LINE_CALIBRATION_MIN_SEPARATION_RAW)
        {
            calibration->fail_mask |= (uint8_t)(1U << channel);
        }
        thresholds[channel] = (uint16_t)(((uint32_t)floor_mean + line_mean) / 2UL);
        floor_total += floor_mean;
        line_total += line_mean;
    }
    if (calibration->fail_mask != 0U)
    {
        return 0U;
    }
    *active_low = (line_total < floor_total) ? 1U : 0U;
    return 1U;
}
