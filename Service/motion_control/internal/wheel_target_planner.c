#include "wheel_target_planner.h"

#include "differential_drive_kinematics.h"

static float WheelTargetPlanner_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float WheelTargetPlanner_RampToward(float current, float target, float step)
{
    if (current < target)
    {
        current += step;
        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        current -= step;
        if (current < target)
        {
            current = target;
        }
    }
    return current;
}

static void WheelTargetPlanner_Resolve(float                linear_x,
                                       float                angular_z,
                                       const param_model_t *params,
                                       float               *left_mps,
                                       float               *right_mps)
{
    DifferentialDriveKinematics_ResolveDifferentialTargets(linear_x,
                                                           angular_z,
                                                           params->track_width_m,
                                                           left_mps,
                                                           right_mps);
}

void WheelTargetPlanner_Init(wheel_target_planner_t *planner, const motion_control_config_t *config)
{
    if (planner != 0 && config != 0)
    {
        *planner                     = (wheel_target_planner_t){0};
        planner->full_speed_mps      = config->open_loop_full_mps;
        planner->angular_epsilon_rps = config->angular_epsilon_rps;
        planner->proportional_scale  = config->wheel_speed_proportional_scale;
        StraightLineController_Init(&planner->straight_line_controller);
    }
}

void WheelTargetPlanner_Reset(wheel_target_planner_t *planner)
{
    if (planner != 0)
    {
        planner->ramped_linear_x  = 0.0f;
        planner->ramped_angular_z = 0.0f;
        StraightLineController_Reset(&planner->straight_line_controller);
    }
}

void WheelTargetPlanner_ScaleWheelTargets(const wheel_target_planner_t *planner, float *left_mps, float *right_mps)
{
    float abs_left;
    float abs_right;
    float max_abs;

    if (planner == 0 || planner->proportional_scale == 0U || left_mps == 0 || right_mps == 0)
    {
        return;
    }
    abs_left  = WheelTargetPlanner_Abs(*left_mps);
    abs_right = WheelTargetPlanner_Abs(*right_mps);
    max_abs   = (abs_left > abs_right) ? abs_left : abs_right;
    if (max_abs > planner->full_speed_mps)
    {
        float scale = planner->full_speed_mps / max_abs;
        *left_mps *= scale;
        *right_mps *= scale;
    }
}

void WheelTargetPlanner_Step(wheel_target_planner_t             *planner,
                             const wheel_target_planner_input_t *input,
                             wheel_target_planner_result_t      *result)
{
    float linear_step;
    float angular_step;

    if (planner == 0 || input == 0 || input->command == 0 || input->params == 0 || result == 0)
    {
        return;
    }
    *result      = (wheel_target_planner_result_t){0};
    linear_step  = input->params->speed_ramp_mps2 * input->dt_s;
    angular_step = input->params->angular_ramp_rps2 * input->dt_s;

    WheelTargetPlanner_Resolve(input->command->linear_x,
                               input->command->angular_z,
                               input->params,
                               &result->requested_left_mps,
                               &result->requested_right_mps);
    WheelTargetPlanner_ScaleWheelTargets(planner, &result->requested_left_mps, &result->requested_right_mps);

    planner->ramped_linear_x =
        WheelTargetPlanner_RampToward(planner->ramped_linear_x, input->command->linear_x, linear_step);
    planner->ramped_angular_z =
        WheelTargetPlanner_RampToward(planner->ramped_angular_z, input->command->angular_z, angular_step);
    WheelTargetPlanner_Resolve(planner->ramped_linear_x,
                               planner->ramped_angular_z,
                               input->params,
                               &result->target_left_mps,
                               &result->target_right_mps);

    if (WheelTargetPlanner_Abs(input->command->angular_z) <= planner->angular_epsilon_rps
        && WheelTargetPlanner_Abs(input->command->linear_x) > 0.001f)
    {
        straight_line_controller_params_t params = {
            .trim_forward_015_mps         = input->params->straight_trim_forward_015_mps,
            .trim_forward_030_mps         = input->params->straight_trim_forward_030_mps,
            .trim_reverse_015_mps         = input->params->straight_trim_reverse_015_mps,
            .trim_reverse_030_mps         = input->params->straight_trim_reverse_030_mps,
            .wheel_coupling_gain          = input->params->straight_wheel_coupling_gain,
            .heading_kp                   = input->params->straight_heading_kp,
            .heading_ki                   = input->params->straight_heading_ki,
            .heading_integral_limit_deg_s = input->params->straight_heading_integral_limit_deg_s,
            .max_speed_mps                = input->params->straight_max_speed_mps,
            .heading_enabled              = input->params->straight_heading_hold_enabled,
        };
        straight_line_controller_input_t straight_input = {
            .now_ms                = input->now_ms,
            .source                = input->command->source,
            .generation            = input->motion_generation,
            .requested_linear_mps  = planner->ramped_linear_x,
            .requested_angular_rps = input->command->angular_z,
            .actual_left_mps       = input->actual_left_mps,
            .actual_right_mps      = input->actual_right_mps,
            .left_speed_valid      = input->left_speed_valid,
            .right_speed_valid     = input->right_speed_valid,
            .left_output_permille  = input->left_output_permille,
            .right_output_permille = input->right_output_permille,
            .left_current_limited  = input->left_current_limited,
            .right_current_limited = input->right_current_limited,
            .imu_valid             = input->imu_valid,
            .gyro_z_dps            = input->gyro_z_dps,
        };

        result->straight = StraightLineController_Step(&planner->straight_line_controller, &params, &straight_input);
        result->target_left_mps  = result->straight.left_target_mps;
        result->target_right_mps = result->straight.right_target_mps;
    }
    else
    {
        StraightLineController_Reset(&planner->straight_line_controller);
    }
    WheelTargetPlanner_ScaleWheelTargets(planner, &result->target_left_mps, &result->target_right_mps);
}
