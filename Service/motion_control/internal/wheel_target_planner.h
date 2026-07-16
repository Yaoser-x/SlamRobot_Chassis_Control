#ifndef CHASSIS_TARGET_PLANNER_H
#define CHASSIS_TARGET_PLANNER_H

#include <stdint.h>

#include "command_management_types.h"
#include "parameter_management_status.h"
#include "motion_control_config.h"
#include "straight_line_controller.h"

typedef struct
{
    float                      ramped_linear_x;
    float                      ramped_angular_z;
    straight_line_controller_t straight_line_controller;
    float                      full_speed_mps;
    float                      angular_epsilon_rps;
    uint8_t                    proportional_scale;
} wheel_target_planner_t;

typedef struct
{
    uint32_t                  now_ms;
    float                     dt_s;
    const command_velocity_t *command;
    const param_model_t      *params;
    uint32_t                  motion_generation;
    float                     actual_left_mps;
    float                     actual_right_mps;
    uint8_t                   left_speed_valid;
    uint8_t                   right_speed_valid;
    int16_t                   left_output_permille;
    int16_t                   right_output_permille;
    uint8_t                   left_current_limited;
    uint8_t                   right_current_limited;
    uint8_t                   imu_valid;
    float                     gyro_z_dps;
} wheel_target_planner_input_t;

typedef struct
{
    float                             requested_left_mps;
    float                             requested_right_mps;
    float                             target_left_mps;
    float                             target_right_mps;
    straight_line_controller_result_t straight;
} wheel_target_planner_result_t;

/** Initialize target ramp and straight-control state. */
void WheelTargetPlanner_Init(wheel_target_planner_t *planner, const motion_control_config_t *config);

/** Reset target ramp and straight-control state. */
void WheelTargetPlanner_Reset(wheel_target_planner_t *planner);

/** Proportionally constrain left and right wheel targets. */
void WheelTargetPlanner_ScaleWheelTargets(const wheel_target_planner_t *planner, float *left_mps, float *right_mps);

/** Resolve one command into requested and ramped side targets. */
void WheelTargetPlanner_Step(wheel_target_planner_t             *planner,
                             const wheel_target_planner_input_t *input,
                             wheel_target_planner_result_t      *result);

#endif
