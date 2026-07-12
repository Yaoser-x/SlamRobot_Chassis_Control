#ifndef ENCODER_MATH_H
#define ENCODER_MATH_H

#include <stdint.h>

#include "bsp_config.h"

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

    int32_t EncoderMath_DiffCount(uint32_t now, uint32_t last, uint32_t period);
    float   EncoderMath_CountDeltaSpeedMps(int32_t delta, uint32_t dt_ms, float counts_per_rev, float wheel_radius_m);
    void    EncoderMath_SpeedWindowReset(encoder_speed_window_t *window);
    void    EncoderMath_SpeedWindowPush(encoder_speed_window_t *window, int32_t delta, uint32_t dt_ms);
    uint8_t EncoderMath_DeltaAccepted(int32_t                       delta,
                                      const encoder_speed_window_t *window,
                                      uint32_t                      dt_ms,
                                      float                         counts_per_rev,
                                      float                         wheel_radius_m,
                                      float                         max_abs_mps,
                                      float                         spike_reject_mps,
                                      uint8_t                       min_samples);
    uint8_t EncoderMath_RecordDeltaOrRebuild(encoder_speed_window_t *window,
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
