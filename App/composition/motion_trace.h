#ifndef APP_MOTION_TRACE_H
#define APP_MOTION_TRACE_H

#include <stdint.h>

#define APP_MOTION_TRACE_CAPACITY     96U
#define APP_MOTION_TRACE_POST_SAMPLES 32U
#define APP_MOTION_TRACE_REVERSE      (1U << 0)
#define APP_MOTION_TRACE_SAFETY       (1U << 1)
#define APP_MOTION_TRACE_MANUAL       (1U << 2)

typedef struct
{
    uint32_t time_ms;
    uint32_t raw_count[4];
    int32_t  count[4];
    int32_t  delta[4];
    int16_t  target_mmps[4];
    int16_t  speed_mmps[4];
    int16_t  requested_pwm[4];
    int16_t  applied_pwm[4];
    int16_t  effective_pwm[4];
    uint8_t  phase[4];
    int8_t   ph_dir[4];
    int8_t   pending_dir[4];
    uint32_t error_flags;
    uint32_t permit_generation;
    uint32_t revoke_generation;
    uint8_t  source;
    uint8_t  enabled_mask;
    uint8_t  safety_state;
    uint8_t  motion_allowed;
    uint8_t  nonzero_target_mask;
    uint8_t  moving_speed_mask;
} app_motion_trace_frame_t;

typedef struct
{
    uint32_t trigger_time_ms;
    uint16_t count;
    uint16_t trigger_index;
    uint8_t  reason;
    uint8_t  frozen;
    uint8_t  post_remaining;
    uint8_t  stationary_samples;
} app_motion_trace_status_t;

void AppMotionTrace_Init(void);
void AppMotionTrace_Record(const app_motion_trace_frame_t *frame);
void AppMotionTrace_GetStatus(app_motion_trace_status_t *status);
/** Explicit rearm/export are accepted only after three stationary control cycles. */
uint8_t AppMotionTrace_Arm(void);
void    AppMotionTrace_Freeze(void);
uint8_t AppMotionTrace_Read(uint16_t index, app_motion_trace_frame_t *frame);

#endif
