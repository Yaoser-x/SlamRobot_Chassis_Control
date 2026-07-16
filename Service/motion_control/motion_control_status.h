#ifndef MOTION_CONTROL_STATUS_H
#define MOTION_CONTROL_STATUS_H

#include <stdint.h>

#define MOTION_CONTROL_MOTOR_COUNT 4U

/** @brief Complete motion feedback, controller, and actuator publication DTO. */
typedef struct
{
    float    motor_target_mps[MOTION_CONTROL_MOTOR_COUNT];
    float    motor_requested_mps[MOTION_CONTROL_MOTOR_COUNT];
    float    motor_actual_mps[MOTION_CONTROL_MOTOR_COUNT];
    float    motor_error_mps[MOTION_CONTROL_MOTOR_COUNT];
    int16_t  motor_output_permille[MOTION_CONTROL_MOTOR_COUNT];
    int16_t  motor_effective_output_permille[MOTION_CONTROL_MOTOR_COUNT];
    uint8_t  motor_speed_valid[MOTION_CONTROL_MOTOR_COUNT];
    uint8_t  motor_pid_active[MOTION_CONTROL_MOTOR_COUNT];
    uint8_t  motor_feedback_lost[MOTION_CONTROL_MOTOR_COUNT];
    uint8_t  motor_current_limited[MOTION_CONTROL_MOTOR_COUNT];
    uint8_t  motor_enabled_mask;
    float    left_target_mps;
    float    right_target_mps;
    float    left_requested_mps;
    float    right_requested_mps;
    float    left_actual_mps;
    float    right_actual_mps;
    float    left_error_mps;
    float    right_error_mps;
    int16_t  left_output_permille;
    int16_t  right_output_permille;
    uint8_t  output_enabled;
    uint8_t  left_speed_valid;
    uint8_t  right_speed_valid;
    uint8_t  left_pid_active;
    uint8_t  right_pid_active;
    uint8_t  left_feedback_lost;
    uint8_t  right_feedback_lost;
    uint8_t  left_current_limited;
    uint8_t  right_current_limited;
    float    straight_wheel_correction_mps;
    float    straight_trim_mps;
    float    straight_heading_error_deg;
    float    straight_heading_integral_deg_s;
    float    straight_heading_correction_mps;
    float    straight_total_correction_mps;
    float    straight_transition_distance_m;
    int8_t   straight_direction;
    uint8_t  straight_active;
    uint8_t  straight_heading_degraded;
    uint8_t  straight_derated;
    uint8_t  straight_out_of_range;
    uint8_t  straight_in_transition;
    uint8_t  pwm_saturated;
    uint8_t  control_source;
    uint32_t generation;
} motion_control_status_t;

#endif /* MOTION_CONTROL_STATUS_H */
