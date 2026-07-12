#include "chassis_math.h"

void ChassisMath_ResolveDifferentialTargets(float  linear_x,
                                            float  angular_z,
                                            float  wheel_base_m,
                                            float *left_mps,
                                            float *right_mps)
{
    float half_track_omega = angular_z * wheel_base_m * 0.5f;

    if (left_mps != 0)
    {
        *left_mps = linear_x - half_track_omega;
    }
    if (right_mps != 0)
    {
        *right_mps = linear_x + half_track_omega;
    }
}

uint8_t ChassisMath_ControlDt(uint32_t now_ms, uint32_t *last_step_ms, uint8_t *initialized, float *dt_s)
{
    uint32_t elapsed_ms;

    if (last_step_ms == 0 || initialized == 0 || dt_s == 0)
    {
        return 0U;
    }
    if (*initialized == 0U)
    {
        *last_step_ms = now_ms;
        *initialized  = 1U;
        *dt_s         = 0.010f;
        return 1U;
    }
    elapsed_ms    = now_ms - *last_step_ms;
    *last_step_ms = now_ms;
    if (elapsed_ms > 100U)
    {
        *dt_s = 0.0f;
        return 0U;
    }
    if (elapsed_ms == 0U)
    {
        elapsed_ms = 1U;
    }
    *dt_s = (float)elapsed_ms / 1000.0f;
    return 1U;
}
