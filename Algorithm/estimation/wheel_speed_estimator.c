#include "wheel_speed_estimator.h"

#define TWO_PI_F                      6.28318530718f
#define WHEEL_REACQUIRING_FLAG        0x80U
#define WHEEL_REACQUIRE_COUNT_MASK    0x7FU
#define WHEEL_REACQUIRE_VALID_SAMPLES 3U

static float WheelSpeedEstimator_AbsF(float value)
{
    return (value < 0.0f) ? -value : value;
}

int32_t WheelSpeedEstimator_DiffCount(uint32_t now, uint32_t last, uint32_t period)
{
    if (period <= 0xFFFFU)
    {
        return (int32_t)(int16_t)((uint16_t)now - (uint16_t)last);
    }
    return (int32_t)(now - last);
}

float WheelSpeedEstimator_CountDeltaSpeedMps(int32_t delta, uint32_t dt_ms, float counts_per_rev, float wheel_radius_m)
{
    if (dt_ms == 0U || counts_per_rev <= 0.0f || wheel_radius_m <= 0.0f)
    {
        return 0.0f;
    }

    return ((float)delta / counts_per_rev) * (TWO_PI_F * wheel_radius_m) / ((float)dt_ms / 1000.0f);
}

void WheelSpeedEstimator_SpeedWindowReset(encoder_speed_window_t *window)
{
    if (window == 0)
    {
        return;
    }

    for (uint32_t i = 0U; i < CHASSIS_ENCODER_SPEED_WINDOW_SAMPLES; ++i)
    {
        window->delta_history[i] = 0;
        window->dt_history_ms[i] = 0U;
    }
    window->delta_sum    = 0;
    window->dt_sum_ms    = 0U;
    window->next_index   = 0U;
    window->sample_count = 0U;
}

void WheelSpeedEstimator_SpeedWindowPush(encoder_speed_window_t *window, int32_t delta, uint32_t dt_ms)
{
    uint8_t index;

    if (window == 0)
    {
        return;
    }

    index = window->next_index;
    window->delta_sum -= window->delta_history[index];
    window->dt_sum_ms -= window->dt_history_ms[index];
    window->delta_history[index] = delta;
    window->dt_history_ms[index] = dt_ms;
    window->delta_sum += delta;
    window->dt_sum_ms += dt_ms;

    window->next_index = (uint8_t)((index + 1U) % CHASSIS_ENCODER_SPEED_WINDOW_SAMPLES);
    if (window->sample_count < CHASSIS_ENCODER_SPEED_WINDOW_SAMPLES)
    {
        window->sample_count++;
    }
}

uint8_t WheelSpeedEstimator_DeltaAccepted(int32_t                       delta,
                                          const encoder_speed_window_t *window,
                                          uint32_t                      dt_ms,
                                          float                         counts_per_rev,
                                          float                         wheel_radius_m,
                                          float                         max_abs_mps,
                                          float                         spike_reject_mps,
                                          uint8_t                       min_samples)
{
    float current_speed;
    float history_speed;

    if (dt_ms == 0U || counts_per_rev <= 0.0f || wheel_radius_m <= 0.0f)
    {
        return 0U;
    }

    current_speed = WheelSpeedEstimator_CountDeltaSpeedMps(delta, dt_ms, counts_per_rev, wheel_radius_m);
    if (max_abs_mps > 0.0f && WheelSpeedEstimator_AbsF(current_speed) > max_abs_mps)
    {
        return 0U;
    }

    if (window == 0 || spike_reject_mps <= 0.0f || window->sample_count < min_samples || window->dt_sum_ms == 0U)
    {
        return 1U;
    }

    history_speed =
        WheelSpeedEstimator_CountDeltaSpeedMps(window->delta_sum, window->dt_sum_ms, counts_per_rev, wheel_radius_m);
    return (WheelSpeedEstimator_AbsF(current_speed - history_speed) <= spike_reject_mps) ? 1U : 0U;
}

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
                                                 uint16_t               *rebuild_count)
{
    uint8_t accepted;
    uint8_t reacquiring = (reject_streak != 0 && (*reject_streak & WHEEL_REACQUIRING_FLAG) != 0U) ? 1U : 0U;

    accepted = WheelSpeedEstimator_DeltaAccepted(delta,
                                                 window,
                                                 dt_ms,
                                                 counts_per_rev,
                                                 wheel_radius_m,
                                                 max_abs_mps,
                                                 spike_reject_mps,
                                                 min_samples);
    if (accepted != 0U)
    {
        WheelSpeedEstimator_SpeedWindowPush(window, delta, dt_ms);
        if (reject_streak != 0)
        {
            if (reacquiring != 0U)
            {
                uint8_t count = (uint8_t)((*reject_streak & WHEEL_REACQUIRE_COUNT_MASK) + 1U);
                if (count < WHEEL_REACQUIRE_VALID_SAMPLES)
                {
                    *reject_streak = (uint8_t)(WHEEL_REACQUIRING_FLAG | count);
                    return 0U;
                }
                if (rebuild_count != 0)
                {
                    (*rebuild_count)++;
                }
            }
            *reject_streak = 0U;
        }
        return 1U;
    }

    if (reject_streak != 0)
    {
        if (reacquiring != 0U)
        {
            WheelSpeedEstimator_SpeedWindowReset(window);
            *reject_streak = WHEEL_REACQUIRING_FLAG;
            return 0U;
        }
        (*reject_streak)++;
        if (rebuild_after_rejects != 0U && *reject_streak >= rebuild_after_rejects)
        {
            WheelSpeedEstimator_SpeedWindowReset(window);
            *reject_streak = WHEEL_REACQUIRING_FLAG;
        }
    }

    return 0U;
}
