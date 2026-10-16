#include "motion_trace.h"
#include "platform_critical.h"

static app_motion_trace_frame_t  frames[APP_MOTION_TRACE_CAPACITY];
static app_motion_trace_frame_t  previous;
static app_motion_trace_status_t trace;
static uint16_t                  next_frame;
static uint8_t                   have_previous;

static uint8_t Stationary(const app_motion_trace_frame_t *frame)
{
    for (uint8_t i = 0; i < 4U; ++i)
    {
        if ((frame->enabled_mask & (1U << i)) != 0U
            && (((frame->nonzero_target_mask | frame->moving_speed_mask) & (1U << i)) != 0U
                || frame->target_mmps[i] != 0 || frame->requested_pwm[i] != 0 || frame->applied_pwm[i] != 0
                || frame->effective_pwm[i] != 0 || frame->speed_mmps[i] > 5 || frame->speed_mmps[i] < -5
                || have_previous == 0U || frame->count[i] != previous.count[i]))
            return 0U;
    }
    return 1U;
}

void AppMotionTrace_Init(void)
{
    trace         = (app_motion_trace_status_t){0};
    next_frame    = 0U;
    have_previous = 0U;
    previous      = (app_motion_trace_frame_t){0};
}

void AppMotionTrace_Record(const app_motion_trace_frame_t *frame)
{
    uint8_t                   reason = 0U;
    platform_critical_state_t critical;
    if (frame == 0)
        return;
    critical = PlatformCritical_Enter();
    if (Stationary(frame) != 0U)
    {
        if (trace.stationary_samples < 3U)
            trace.stationary_samples++;
    }
    else
        trace.stationary_samples = 0U;
    if (trace.frozen == 0U)
    {
        frames[next_frame] = *frame;
        next_frame         = (uint16_t)((next_frame + 1U) % APP_MOTION_TRACE_CAPACITY);
        if (trace.count < APP_MOTION_TRACE_CAPACITY)
            trace.count++;
        if (trace.reason == 0U && have_previous != 0U)
        {
            for (uint8_t i = 0; i < 4U; ++i)
            {
                if ((frame->enabled_mask & (1U << i)) != 0U
                    && ((frame->target_mmps[i] > 0 && previous.target_mmps[i] > 0 && frame->delta[i] < 0)
                        || (frame->target_mmps[i] < 0 && previous.target_mmps[i] < 0 && frame->delta[i] > 0)))
                    reason |= APP_MOTION_TRACE_REVERSE;
            }
            if (frame->error_flags != previous.error_flags && frame->error_flags != 0U)
            {
                for (uint8_t i = 0; i < 4U; ++i)
                    if (previous.target_mmps[i] != 0 || previous.effective_pwm[i] != 0)
                        reason |= APP_MOTION_TRACE_SAFETY;
            }
            if (reason != 0U)
            {
                trace.reason          = reason;
                trace.trigger_time_ms = frame->time_ms;
                trace.post_remaining  = APP_MOTION_TRACE_POST_SAMPLES;
            }
        }
        else if (trace.post_remaining != 0U)
        {
            trace.post_remaining--;
            if (trace.post_remaining == 0U)
                trace.frozen = 1U;
        }
    }
    previous      = *frame;
    have_previous = 1U;
    PlatformCritical_Exit(critical);
}

void AppMotionTrace_GetStatus(app_motion_trace_status_t *status)
{
    if (status != 0)
    {
        platform_critical_state_t critical = PlatformCritical_Enter();
        *status                            = trace;
        status->trigger_index =
            (trace.reason != 0U) ? (uint16_t)(trace.count - 1U - (APP_MOTION_TRACE_POST_SAMPLES - trace.post_remaining))
                                 : 0U;
        if (trace.reason == APP_MOTION_TRACE_MANUAL)
            status->trigger_index = trace.count ? trace.count - 1U : 0U;
        PlatformCritical_Exit(critical);
    }
}

uint8_t AppMotionTrace_Arm(void)
{
    platform_critical_state_t critical = PlatformCritical_Enter();
    uint8_t                   accepted = (trace.stationary_samples >= 3U) ? 1U : 0U;
    if (accepted != 0U)
    {
        uint8_t stationary = trace.stationary_samples;
        AppMotionTrace_Init();
        trace.stationary_samples = stationary;
    }
    PlatformCritical_Exit(critical);
    return accepted;
}

void AppMotionTrace_Freeze(void)
{
    platform_critical_state_t critical = PlatformCritical_Enter();
    if (trace.reason == 0U)
    {
        trace.reason          = APP_MOTION_TRACE_MANUAL;
        trace.trigger_time_ms = previous.time_ms;
        trace.frozen          = 1U;
    }
    PlatformCritical_Exit(critical);
}

uint8_t AppMotionTrace_Read(uint16_t index, app_motion_trace_frame_t *frame)
{
    uint8_t                   accepted = 0U;
    platform_critical_state_t critical = PlatformCritical_Enter();
    if (frame != 0 && trace.frozen != 0U && trace.stationary_samples >= 3U && index < trace.count)
    {
        uint16_t oldest =
            (uint16_t)((next_frame + APP_MOTION_TRACE_CAPACITY - trace.count) % APP_MOTION_TRACE_CAPACITY);
        *frame   = frames[(oldest + index) % APP_MOTION_TRACE_CAPACITY];
        accepted = 1U;
    }
    PlatformCritical_Exit(critical);
    return accepted;
}
