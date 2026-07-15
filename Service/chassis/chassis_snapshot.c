#include "chassis_snapshot.h"

#include "chassis_layout.h"

static float ChassisSnapshot_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

void ChassisSnapshot_SetSideTargets(chassis_service_snapshot_t *snapshot,
                                    float                       left_mps,
                                    float                       right_mps,
                                    uint8_t                     requested)
{
    if (snapshot == 0)
    {
        return;
    }
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        motor_id_t motor  = (motor_id_t)index;
        float      target = 0.0f;

        if (ChassisLayout_MotorEnabled(motor) != 0U)
        {
            target = (ChassisLayout_MotorSide(motor) == MOTOR_SIDE_LEFT) ? left_mps : right_mps;
        }
        snapshot->motor_target_mps[index] = target;
        if (requested != 0U)
        {
            snapshot->motor_requested_mps[index] = target;
        }
    }
}

void ChassisSnapshot_SyncSides(chassis_service_snapshot_t *snapshot)
{
    uint8_t left_count          = 0U;
    uint8_t right_count         = 0U;
    float   left_target_sum     = 0.0f;
    float   right_target_sum    = 0.0f;
    float   left_requested_sum  = 0.0f;
    float   right_requested_sum = 0.0f;
    float   left_actual_sum     = 0.0f;
    float   right_actual_sum    = 0.0f;
    float   left_error_sum      = 0.0f;
    float   right_error_sum     = 0.0f;
    int32_t left_output_sum     = 0;
    int32_t right_output_sum    = 0;

    if (snapshot == 0)
    {
        return;
    }
    snapshot->left_speed_valid      = (ChassisLayout_SideMotorCount(MOTOR_SIDE_LEFT) != 0U) ? 1U : 0U;
    snapshot->right_speed_valid     = (ChassisLayout_SideMotorCount(MOTOR_SIDE_RIGHT) != 0U) ? 1U : 0U;
    snapshot->left_pid_active       = 0U;
    snapshot->right_pid_active      = 0U;
    snapshot->left_feedback_lost    = 0U;
    snapshot->right_feedback_lost   = 0U;
    snapshot->left_current_limited  = 0U;
    snapshot->right_current_limited = 0U;

    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        motor_id_t motor = (motor_id_t)index;
        if (ChassisLayout_MotorEnabled(motor) == 0U)
        {
            continue;
        }
        if (ChassisLayout_MotorSide(motor) == MOTOR_SIDE_LEFT)
        {
            left_count++;
            left_target_sum += snapshot->motor_target_mps[index];
            left_requested_sum += snapshot->motor_requested_mps[index];
            left_actual_sum += snapshot->motor_actual_mps[index];
            left_error_sum += snapshot->motor_error_mps[index];
            left_output_sum += snapshot->motor_output_permille[index];
            if (snapshot->motor_speed_valid[index] == 0U)
            {
                snapshot->left_speed_valid = 0U;
            }
            snapshot->left_pid_active |= snapshot->motor_pid_active[index];
            snapshot->left_feedback_lost |= snapshot->motor_feedback_lost[index];
            snapshot->left_current_limited |= snapshot->motor_current_limited[index];
        }
        else
        {
            right_count++;
            right_target_sum += snapshot->motor_target_mps[index];
            right_requested_sum += snapshot->motor_requested_mps[index];
            right_actual_sum += snapshot->motor_actual_mps[index];
            right_error_sum += snapshot->motor_error_mps[index];
            right_output_sum += snapshot->motor_output_permille[index];
            if (snapshot->motor_speed_valid[index] == 0U)
            {
                snapshot->right_speed_valid = 0U;
            }
            snapshot->right_pid_active |= snapshot->motor_pid_active[index];
            snapshot->right_feedback_lost |= snapshot->motor_feedback_lost[index];
            snapshot->right_current_limited |= snapshot->motor_current_limited[index];
        }
    }
    snapshot->left_target_mps       = (left_count != 0U) ? (left_target_sum / (float)left_count) : 0.0f;
    snapshot->right_target_mps      = (right_count != 0U) ? (right_target_sum / (float)right_count) : 0.0f;
    snapshot->left_requested_mps    = (left_count != 0U) ? (left_requested_sum / (float)left_count) : 0.0f;
    snapshot->right_requested_mps   = (right_count != 0U) ? (right_requested_sum / (float)right_count) : 0.0f;
    snapshot->left_actual_mps       = (left_count != 0U) ? (left_actual_sum / (float)left_count) : 0.0f;
    snapshot->right_actual_mps      = (right_count != 0U) ? (right_actual_sum / (float)right_count) : 0.0f;
    snapshot->left_error_mps        = (left_count != 0U) ? (left_error_sum / (float)left_count) : 0.0f;
    snapshot->right_error_mps       = (right_count != 0U) ? (right_error_sum / (float)right_count) : 0.0f;
    snapshot->left_output_permille  = (left_count != 0U) ? (int16_t)(left_output_sum / (int32_t)left_count) : 0;
    snapshot->right_output_permille = (right_count != 0U) ? (int16_t)(right_output_sum / (int32_t)right_count) : 0;
}

void ChassisSnapshot_ApplyPlannerResult(chassis_service_snapshot_t            *snapshot,
                                        const chassis_target_planner_result_t *result,
                                        uint8_t                                control_source)
{
    if (snapshot == 0 || result == 0)
    {
        return;
    }
    snapshot->straight_active                 = result->straight.active;
    snapshot->straight_direction              = result->straight.direction;
    snapshot->straight_transition_distance_m  = result->straight.transition_distance_m;
    snapshot->straight_in_transition          = result->straight.in_transition;
    snapshot->straight_trim_mps               = result->straight.trim_correction_mps;
    snapshot->straight_wheel_correction_mps   = result->straight.wheel_correction_mps;
    snapshot->straight_heading_error_deg      = result->straight.heading_error_deg;
    snapshot->straight_heading_integral_deg_s = result->straight.heading_integral_deg_s;
    snapshot->straight_heading_correction_mps = result->straight.heading_correction_mps;
    snapshot->straight_total_correction_mps   = result->straight.total_correction_mps;
    snapshot->straight_heading_degraded       = result->straight.heading_degraded;
    snapshot->straight_derated                = result->straight.derated;
    snapshot->straight_out_of_range           = result->straight.out_of_range;
    snapshot->control_source                  = control_source;
    snapshot->pwm_saturated = (uint8_t)(ChassisSnapshot_Abs((float)snapshot->left_output_permille) >= 850.0f
                                        || ChassisSnapshot_Abs((float)snapshot->right_output_permille) >= 850.0f);
}
