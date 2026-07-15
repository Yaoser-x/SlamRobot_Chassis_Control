#include "chassis_feedback_guard.h"

#include "chassis_layout.h"
#include "control_config.h"

static float ChassisFeedbackGuard_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

void ChassisFeedbackGuard_Init(chassis_feedback_guard_t *guard)
{
    if (guard != 0)
    {
        *guard = (chassis_feedback_guard_t){0};
    }
}

void ChassisFeedbackGuard_Reset(chassis_feedback_guard_t *guard, chassis_service_snapshot_t *snapshot)
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

uint8_t ChassisFeedbackGuard_DetectFault(chassis_feedback_guard_t   *guard,
                                         uint32_t                    now_ms,
                                         chassis_service_snapshot_t *snapshot,
                                         const encoder_state_t      *encoder,
                                         const motor_driver_state_t *motor)
{
    if (guard == 0 || snapshot == 0 || encoder == 0 || motor == 0)
    {
        return 0U;
    }
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        if (ChassisLayout_MotorEnabled((motor_id_t)index) == 0U)
        {
            guard->no_motion_active[index] = 0U;
            continue;
        }
        if (encoder->speed_valid[index] == 0U)
        {
            snapshot->motor_feedback_lost[index] = 1U;
            return 1U;
        }
        if (ChassisFeedbackGuard_Abs(snapshot->motor_requested_mps[index]) < CHASSIS_PID_FEEDBACK_MIN_TARGET_MPS
            || ChassisFeedbackGuard_Abs(encoder->speed_mps[index]) >= CHASSIS_PID_FEEDBACK_MIN_SPEED_MPS)
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
        else if ((uint32_t)(now_ms - guard->no_motion_since_ms[index]) >= CHASSIS_ENCODER_FEEDBACK_TIMEOUT_MS)
        {
            snapshot->motor_feedback_lost[index] = 1U;
            return 1U;
        }
    }
    return 0U;
}

uint8_t ChassisFeedbackGuard_CheckUsable(chassis_feedback_guard_t   *guard,
                                         chassis_service_snapshot_t *snapshot,
                                         motor_id_t                  motor,
                                         float                       target_mps,
                                         float                       actual_mps,
                                         uint8_t                     encoder_valid)
{
    if (guard == 0 || snapshot == 0 || (uint32_t)motor >= MOTOR_ID_COUNT)
    {
        return 0U;
    }
    snapshot->motor_feedback_lost[motor] = 0U;
    if (encoder_valid == 0U)
    {
        guard->loss_count[motor]             = CHASSIS_PID_FEEDBACK_LOSS_COUNT;
        snapshot->motor_feedback_lost[motor] = 1U;
        return 0U;
    }
    if (ChassisFeedbackGuard_Abs(target_mps) < CHASSIS_PID_FEEDBACK_MIN_TARGET_MPS
        || ChassisFeedbackGuard_Abs(actual_mps) >= CHASSIS_PID_FEEDBACK_MIN_SPEED_MPS)
    {
        guard->loss_count[motor] = 0U;
        return 1U;
    }
    if (guard->loss_count[motor] < CHASSIS_PID_FEEDBACK_LOSS_COUNT)
    {
        guard->loss_count[motor]++;
    }
    if (guard->loss_count[motor] >= CHASSIS_PID_FEEDBACK_LOSS_COUNT)
    {
        snapshot->motor_feedback_lost[motor] = 1U;
        return 0U;
    }
    return 1U;
}
