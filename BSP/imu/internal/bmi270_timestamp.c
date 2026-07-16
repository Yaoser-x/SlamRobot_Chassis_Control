#include "bmi270_timestamp.h"

uint32_t ImuBmi270Time_DeltaTicks24(uint32_t now, uint32_t previous)
{
    return (now - previous) & IMU_BMI270_SENSOR_TIME_MASK;
}

uint8_t ImuBmi270Time_DeltaSeconds(uint32_t now, uint32_t previous, float max_dt_s, float *dt_s)
{
    uint32_t delta_ticks;
    float    seconds;

    if (dt_s == 0)
    {
        return 0U;
    }

    delta_ticks = ImuBmi270Time_DeltaTicks24(now, previous);
    if (delta_ticks == 0UL)
    {
        return 0U;
    }

    seconds = (float)delta_ticks / IMU_BMI270_SENSOR_TIME_TICKS_PER_SEC;
    if (seconds <= 0.0f || seconds > max_dt_s)
    {
        return 0U;
    }

    *dt_s = seconds;
    return 1U;
}
