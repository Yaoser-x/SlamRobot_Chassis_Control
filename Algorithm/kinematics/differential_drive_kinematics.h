#ifndef CHASSIS_MATH_H
#define CHASSIS_MATH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        CONTROL_TIMING_FIRST = 0,
        CONTROL_TIMING_EARLY,
        CONTROL_TIMING_NORMAL,
        CONTROL_TIMING_LATE,
        CONTROL_TIMING_MISSED
    } control_timing_status_t;

    typedef struct
    {
        uint32_t                last_step_ms;
        uint32_t                late_count;
        uint32_t                missed_count;
        uint8_t                 initialized;
        control_timing_status_t status;
        float                   dt_s;
    } control_timing_t;

    void DifferentialDriveKinematics_ResolveDifferentialTargets(float  linear_x,
                                                                float  angular_z,
                                                                float  track_width_m,
                                                                float *left_mps,
                                                                float *right_mps);
    uint8_t
    DifferentialDriveKinematics_ControlDt(uint32_t now_ms, uint32_t *last_step_ms, uint8_t *initialized, float *dt_s);
    control_timing_status_t DifferentialDriveKinematics_EvaluateControlTiming(control_timing_t *timing,
                                                                              uint32_t          now_ms,
                                                                              uint32_t          nominal_period_ms);

#ifdef __cplusplus
}
#endif

#endif
