#include "wheel_target_planner.h"
#include "control_config.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void AssertNear(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

int main(void)
{
    wheel_target_planner_t        planner;
    wheel_target_planner_input_t  input = {0};
    wheel_target_planner_result_t result;
    param_model_t                 params = {0};
    command_velocity_t command = {.linear_x = 0.3f, .angular_z = 0.2f, .enable = 1U, .source = COMMAND_SOURCE_HOST};
    const motion_control_config_t config = {
        .max_linear_mps                 = CHASSIS_MAX_LINEAR_MPS,
        .open_loop_full_mps             = CHASSIS_OPENLOOP_FULL_MPS,
        .angular_epsilon_rps            = CHASSIS_ANGULAR_EPSILON_RPS,
        .wheel_speed_proportional_scale = CHASSIS_WHEEL_SPEED_PROPORTIONAL_SCALE,
    };
    float left  = 2.0f;
    float right = 1.0f;

    params.track_width_m     = 0.4f;
    params.speed_ramp_mps2   = 1.0f;
    params.angular_ramp_rps2 = 1.0f;
    input.dt_s               = 0.1f;
    input.command            = &command;
    input.params             = &params;

    WheelTargetPlanner_Init(&planner, &config);
    WheelTargetPlanner_Step(&planner, &input, &result);
    AssertNear(result.requested_left_mps, 0.26f);
    AssertNear(result.requested_right_mps, 0.34f);
    AssertNear(result.target_left_mps, 0.08f);
    AssertNear(result.target_right_mps, 0.12f);

    WheelTargetPlanner_Reset(&planner);
    WheelTargetPlanner_Step(&planner, &input, &result);
    AssertNear(result.target_left_mps, 0.08f);
    AssertNear(result.target_right_mps, 0.12f);

    WheelTargetPlanner_ScaleWheelTargets(&planner, &left, &right);
    assert(fabsf(left) <= CHASSIS_OPENLOOP_FULL_MPS + 0.0001f);
    AssertNear(right / left, 0.5f);

    puts("PASS: chassis target planner");
    return 0;
}
