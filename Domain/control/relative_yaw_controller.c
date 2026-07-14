#include "relative_yaw_controller.h"

#define RELATIVE_YAW_KP_PER_S        2.0f
#define RELATIVE_YAW_MAX_RPS         1.5f
#define RELATIVE_YAW_TOLERANCE_DEG   2.0f
#define RELATIVE_YAW_SETTLE_RATE_DPS 5.0f
#define RELATIVE_YAW_SETTLE_MS       100U
#define DEG_TO_RAD_F                 0.01745329252f

static float RelativeYawControl_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

void RelativeYawControl_Init(relative_yaw_control_t *control)
{
    if (control != 0)
    {
        *control = (relative_yaw_control_t){0};
    }
}

uint8_t RelativeYawControl_Start(relative_yaw_control_t *control,
                                 float                   target_delta_deg,
                                 float                   initial_yaw_deg,
                                 uint32_t                now_ms,
                                 uint32_t                timeout_ms)
{
    if (control == 0 || timeout_ms == 0U || target_delta_deg == 0.0f)
    {
        return 0U;
    }

    *control                  = (relative_yaw_control_t){0};
    control->active           = 1U;
    control->target_delta_deg = target_delta_deg;
    control->last_update_ms   = now_ms;
    control->start_ms         = now_ms;
    control->timeout_ms       = timeout_ms;
    (void)initial_yaw_deg;
    return 1U;
}

void RelativeYawControl_Cancel(relative_yaw_control_t *control, relative_yaw_end_reason_t reason)
{
    if (control != 0)
    {
        control->active     = 0U;
        control->settling   = 0U;
        control->end_reason = reason;
    }
}

uint8_t RelativeYawControl_Update(relative_yaw_control_t *control,
                                  float                   yaw_deg,
                                  float                   yaw_rate_dps,
                                  uint32_t                now_ms,
                                  float                  *angular_z)
{
    float    error_deg;
    float    command;
    float    dt_s;
    uint32_t dt_ms;

    if (angular_z != 0)
    {
        *angular_z = 0.0f;
    }
    if (control == 0 || angular_z == 0 || control->active == 0U)
    {
        return 0U;
    }
    if ((uint32_t)(now_ms - control->start_ms) >= control->timeout_ms)
    {
        RelativeYawControl_Cancel(control, RELATIVE_YAW_END_TIMEOUT);
        return 0U;
    }

    /* 用陀螺仪速率直接积分累积航向变化，绕过 Mahony 滤波器 yaw 欠报问题。
     实测 Mahony yaw 在手动 360° 旋转时丢失约 40% 旋转量，
     而 gyro_z 积分误差 <1%。 */
    dt_ms = (uint32_t)(now_ms - control->last_update_ms);
    if (dt_ms == 0U || dt_ms > 500U)
    {
        dt_s = 0.02f; /* fallback: PS2 task 50Hz */
    }
    else
    {
        dt_s = (float)dt_ms * 0.001f;
    }
    control->last_update_ms = now_ms;
    control->accumulated_delta_deg += yaw_rate_dps * dt_s;
    (void)yaw_deg;

    error_deg = control->target_delta_deg - control->accumulated_delta_deg;

    if (RelativeYawControl_Abs(error_deg) <= RELATIVE_YAW_TOLERANCE_DEG
        && RelativeYawControl_Abs(yaw_rate_dps) <= RELATIVE_YAW_SETTLE_RATE_DPS)
    {
        if (control->settling == 0U)
        {
            control->settling        = 1U;
            control->settle_start_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - control->settle_start_ms) >= RELATIVE_YAW_SETTLE_MS)
        {
            RelativeYawControl_Cancel(control, RELATIVE_YAW_END_COMPLETED);
            return 0U;
        }
    }
    else
    {
        control->settling = 0U;
    }

    command = RELATIVE_YAW_KP_PER_S * error_deg * DEG_TO_RAD_F;
    if (command > RELATIVE_YAW_MAX_RPS)
    {
        command = RELATIVE_YAW_MAX_RPS;
    }
    else if (command < -RELATIVE_YAW_MAX_RPS)
    {
        command = -RELATIVE_YAW_MAX_RPS;
    }
    *angular_z = command;
    return 1U;
}
