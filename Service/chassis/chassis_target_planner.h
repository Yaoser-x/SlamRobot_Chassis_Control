#ifndef CHASSIS_TARGET_PLANNER_H
#define CHASSIS_TARGET_PLANNER_H

#include <stdint.h>

#include "control_service.h"
#include "param_service.h"
#include "straight_controller.h"

typedef struct
{
    float                 ramped_linear_x;
    float                 ramped_angular_z;
    straight_controller_t straight_controller;
} chassis_target_planner_t;

typedef struct
{
    uint32_t             now_ms;
    float                dt_s;
    const chassis_cmd_t *command;
    const param_model_t *params;
    uint32_t             motion_generation;
    float                actual_left_mps;
    float                actual_right_mps;
    uint8_t              left_speed_valid;
    uint8_t              right_speed_valid;
    int16_t              left_output_permille;
    int16_t              right_output_permille;
    uint8_t              left_current_limited;
    uint8_t              right_current_limited;
    uint8_t              imu_valid;
    float                gyro_z_dps;
} chassis_target_planner_input_t;

typedef struct
{
    float                        requested_left_mps;
    float                        requested_right_mps;
    float                        target_left_mps;
    float                        target_right_mps;
    straight_controller_result_t straight;
} chassis_target_planner_result_t;

/** Initialize target ramp and straight-control state. */
void ChassisTargetPlanner_Init(chassis_target_planner_t *planner);

/** Reset target ramp and straight-control state. */
void ChassisTargetPlanner_Reset(chassis_target_planner_t *planner);

/** Proportionally constrain left and right wheel targets. */
void ChassisTargetPlanner_ScaleWheelTargets(float *left_mps, float *right_mps);

/** Resolve one command into requested and ramped side targets. */
void ChassisTargetPlanner_Step(chassis_target_planner_t             *planner,
                               const chassis_target_planner_input_t *input,
                               chassis_target_planner_result_t      *result);

#endif
