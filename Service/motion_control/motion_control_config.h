#ifndef MOTION_CONTROL_CONFIG_H
#define MOTION_CONTROL_CONFIG_H

#include <stdint.h>

/** @brief Product configuration consumed by Motion Control. */
typedef struct
{
    float    max_linear_mps;
    float    max_angular_rps;
    float    open_loop_full_mps;
    float    angular_epsilon_rps;
    float    speed_ramp_mps2;
    float    angular_ramp_rps2;
    float    maintenance_max_speed_mps;
    float    pid_correction_limit;
    float    pid_stop_epsilon_mps;
    float    pid_direction_epsilon_mps;
    float    pid_feedback_min_target_mps;
    float    pid_feedback_min_speed_mps;
    uint32_t test_mode_lease_ms;
    uint32_t encoder_feedback_timeout_ms;
    uint8_t  pid_feedback_loss_count;
    uint8_t  pid_enabled;
    uint8_t  wheel_speed_proportional_scale;
    uint8_t  motor_current_limiter_observe_only;
    uint8_t  current_soft_limit_enabled;
} motion_control_config_t;

#endif
