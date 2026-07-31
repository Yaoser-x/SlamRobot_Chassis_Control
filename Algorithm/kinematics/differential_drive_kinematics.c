#include "differential_drive_kinematics.h"

static uint8_t DifferentialDriveKinematics_Finite(float value)
{
    const float max_float = 3.402823466e+38f;
    return (value == value && value <= max_float && value >= -max_float) ? 1U : 0U;
}

void DifferentialDriveKinematics_ResolveDifferentialTargets(float  linear_x,
                                                            float  angular_z,
                                                            float  track_width_m,
                                                            float *left_mps,
                                                            float *right_mps)
{
    float half_track_omega;

    if (DifferentialDriveKinematics_Finite(linear_x) == 0U || DifferentialDriveKinematics_Finite(angular_z) == 0U
        || DifferentialDriveKinematics_Finite(track_width_m) == 0U || track_width_m <= 0.0f)
    {
        if (left_mps != 0)
        {
            *left_mps = 0.0f;
        }
        if (right_mps != 0)
        {
            *right_mps = 0.0f;
        }
        return;
    }
    half_track_omega = angular_z * track_width_m * 0.5f;

    if (left_mps != 0)
    {
        *left_mps = linear_x - half_track_omega;
    }
    if (right_mps != 0)
    {
        *right_mps = linear_x + half_track_omega;
    }
}

uint8_t
DifferentialDriveKinematics_ControlDt(uint32_t now_ms, uint32_t *last_step_ms, uint8_t *initialized, float *dt_s)
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

control_timing_status_t
DifferentialDriveKinematics_EvaluateControlTiming(control_timing_t *timing, uint32_t now_ms, uint32_t nominal_period_ms)
{
    uint32_t elapsed_ms;
    uint64_t elapsed_twice;
    uint64_t period;

    if (timing == 0 || nominal_period_ms == 0UL)
    {
        return CONTROL_TIMING_MISSED;
    }
    if (timing->initialized == 0U)
    {
        timing->last_step_ms = now_ms;
        timing->initialized  = 1U;
        timing->status       = CONTROL_TIMING_FIRST;
        timing->dt_s         = 0.0f;
        return timing->status;
    }
    elapsed_ms    = now_ms - timing->last_step_ms;
    elapsed_twice = (uint64_t)elapsed_ms * 2ULL;
    period        = (uint64_t)nominal_period_ms;
    if (elapsed_twice < period)
    {
        timing->status = CONTROL_TIMING_EARLY;
        timing->dt_s   = 0.0f;
        return timing->status;
    }
    timing->last_step_ms = now_ms;
    if ((uint64_t)elapsed_ms > period * 2ULL)
    {
        timing->status = CONTROL_TIMING_MISSED;
        timing->dt_s   = 0.0f;
        timing->missed_count++;
        return timing->status;
    }
    timing->dt_s = (float)elapsed_ms / 1000.0f;
    if (elapsed_twice > period * 3ULL)
    {
        timing->status = CONTROL_TIMING_LATE;
        timing->late_count++;
    }
    else
    {
        timing->status = CONTROL_TIMING_NORMAL;
    }
    return timing->status;
}
