#ifndef WHEEL_SPEED_ESTIMATOR_H
#define WHEEL_SPEED_ESTIMATOR_H

#include <stdint.h>

#ifndef CHASSIS_ENCODER_SPEED_WINDOW_SAMPLES
#define CHASSIS_ENCODER_SPEED_WINDOW_SAMPLES 5U
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        int32_t  delta_history[CHASSIS_ENCODER_SPEED_WINDOW_SAMPLES];
        uint32_t dt_history_ms[CHASSIS_ENCODER_SPEED_WINDOW_SAMPLES];
        int32_t  delta_sum;
        uint32_t dt_sum_ms;
        uint8_t  next_index;
        uint8_t  sample_count;
    } encoder_speed_window_t;

    int32_t WheelSpeedEstimator_DiffCount(uint32_t now, uint32_t last, uint32_t period);
    float
    WheelSpeedEstimator_CountDeltaSpeedMps(int32_t delta, uint32_t dt_ms, float counts_per_rev, float wheel_radius_m);
    void    WheelSpeedEstimator_SpeedWindowReset(encoder_speed_window_t *window);
    void    WheelSpeedEstimator_SpeedWindowPush(encoder_speed_window_t *window, int32_t delta, uint32_t dt_ms);
    uint8_t WheelSpeedEstimator_DeltaAccepted(int32_t                       delta,
                                              const encoder_speed_window_t *window,
                                              uint32_t                      dt_ms,
                                              float                         counts_per_rev,
                                              float                         wheel_radius_m,
                                              float                         max_abs_mps,
                                              float                         spike_reject_mps,
                                              uint8_t                       min_samples);
    uint8_t WheelSpeedEstimator_RecordDeltaOrRebuild(encoder_speed_window_t *window,
                                                     int32_t                 delta,
                                                     uint32_t                dt_ms,
                                                     float                   counts_per_rev,
                                                     float                   wheel_radius_m,
                                                     float                   max_abs_mps,
                                                     float                   spike_reject_mps,
                                                     uint8_t                 min_samples,
                                                     uint8_t                 rebuild_after_rejects,
                                                     uint8_t                *reject_streak,
                                                     uint16_t               *rebuild_count);

#ifdef __cplusplus
}
#endif

#endif
