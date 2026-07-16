#include "bmi270_signal_filter.h"

float ImuSignalFilter_LowPass(float previous, float input, float alpha)
{
    return previous + (alpha * (input - previous));
}

float ImuSignalFilter_WrapAngleDeg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg <= -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}
