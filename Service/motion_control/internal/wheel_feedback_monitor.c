#include "wheel_feedback_monitor.h"

#include "motor_hardware_layout.h"

static float WheelFeedbackMonitor_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

void WheelFeedbackMonitor_Init(wheel_feedback_monitor_t *guard, const motion_control_config_t *config)
{
    if (guard != 0 && config != 0)
    {
        *guard                     = (wheel_feedback_monitor_t){0};
        guard->min_target_mps      = config->pid_feedback_min_target_mps;
        guard->min_speed_mps       = config->pid_feedback_min_speed_mps;
        guard->feedback_timeout_ms = config->encoder_feedback_timeout_ms;
        guard->feedback_loss_count = config->pid_feedback_loss_count;
    }
}

void WheelFeedbackMonitor_Reset(wheel_feedback_monitor_t *guard, motion_control_status_t *snapshot)
{
    if (guard == 0 || snapshot == 0)
    {
        return;
    }
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        guard->loss_count[index]             = 0U;
        guard->no_motion_since_ms[index]     = 0U;
        guard->no_motion_active[index]       = 0U;
        snapshot->motor_feedback_lost[index] = 0U;
    }
    snapshot->left_feedback_lost  = 0U;
    snapshot->right_feedback_lost = 0U;
}

uint8_t WheelFeedbackMonitor_DetectFault(wheel_feedback_monitor_t              *guard,
                                         uint32_t                               now_ms,
                                         motion_control_status_t               *snapshot,
                                         const state_estimation_wheel_status_t *encoder,
                                         const motor_driver_state_t            *motor)
{
    if (guard == 0 || snapshot == 0 || encoder == 0 || motor == 0)
    {
        return 0U;
    }
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        if (MotorHardwareLayout_MotorEnabled((motor_id_t)index) == 0U)
        {
            guard->no_motion_active[index] = 0U;
            continue;
        }
        if (encoder->speed_valid[index] == 0U)
        {
            snapshot->motor_feedback_lost[index] = 1U;
            return 1U;
        }
        if (WheelFeedbackMonitor_Abs(snapshot->motor_requested_mps[index]) < guard->min_target_mps
            || WheelFeedbackMonitor_Abs(encoder->speed_mps[index]) >= guard->min_speed_mps)
        {
            guard->no_motion_active[index] = 0U;
            continue;
        }
        if (motor->phase[index] != MOTOR_DRIVER_PHASE_RUN || motor->effective_pwm[index] == 0)
        {
            guard->no_motion_active[index] = 0U;
            continue;
        }
        if (guard->no_motion_active[index] == 0U)
        {
            guard->no_motion_active[index]   = 1U;
            guard->no_motion_since_ms[index] = now_ms;
        }
        else if ((uint32_t)(now_ms - guard->no_motion_since_ms[index]) >= guard->feedback_timeout_ms)
        {
            snapshot->motor_feedback_lost[index] = 1U;
            return 1U;
        }
    }
    return 0U;
}

uint8_t WheelFeedbackMonitor_CheckUsable(wheel_feedback_monitor_t *guard,
                                         motion_control_status_t  *snapshot,
                                         motor_id_t                motor,
                                         float                     target_mps,
                                         float                     actual_mps,
                                         uint8_t                   encoder_valid)
{
    if (guard == 0 || snapshot == 0 || (uint32_t)motor >= MOTOR_ID_COUNT)
    {
        return 0U;
    }
    snapshot->motor_feedback_lost[motor] = 0U;
    if (encoder_valid == 0U)
    {
        guard->loss_count[motor]             = guard->feedback_loss_count;
        snapshot->motor_feedback_lost[motor] = 1U;
        return 0U;
    }
    if (WheelFeedbackMonitor_Abs(target_mps) < guard->min_target_mps
        || WheelFeedbackMonitor_Abs(actual_mps) >= guard->min_speed_mps)
    {
        guard->loss_count[motor] = 0U;
        return 1U;
    }
    if (guard->loss_count[motor] < guard->feedback_loss_count)
    {
        guard->loss_count[motor]++;
    }
    if (guard->loss_count[motor] >= guard->feedback_loss_count)
    {
        snapshot->motor_feedback_lost[motor] = 1U;
        return 0U;
    }
    return 1U;
}
