#ifndef STRAIGHT_CONTROLLER_H
#define STRAIGHT_CONTROLLER_H

#include <stdint.h>

typedef struct
{
  float trim_forward_015_mps;
  float trim_forward_030_mps;
  float trim_reverse_015_mps;
  float trim_reverse_030_mps;
  float wheel_coupling_gain;
  float heading_kp;
  float heading_ki;
  float heading_integral_limit_deg_s;
  float max_speed_mps;
  uint8_t heading_enabled;
} straight_controller_params_t;

typedef struct
{
  uint32_t now_ms;
  uint8_t source;
  uint32_t generation;
  float requested_linear_mps;
  float requested_angular_rps;
  float actual_left_mps;
  float actual_right_mps;
  uint8_t left_speed_valid;
  uint8_t right_speed_valid;
  int16_t left_output_permille;
  int16_t right_output_permille;
  uint8_t left_current_limited;
  uint8_t right_current_limited;
  uint8_t imu_valid;
  float gyro_z_dps;
} straight_controller_input_t;

typedef struct
{
  float left_target_mps;
  float right_target_mps;
  float transition_distance_m;
  float trim_correction_mps;
  float wheel_correction_mps;
  float heading_error_deg;
  float heading_integral_deg_s;
  float heading_correction_mps;
  float total_correction_mps;
  int8_t direction;
  uint8_t active;
  uint8_t heading_degraded;
  uint8_t derated;
  uint8_t in_transition;
  uint8_t correction_clamped;
} straight_controller_result_t;

typedef struct
{
  uint32_t last_ms;
  uint32_t generation;
  uint8_t source;
  int8_t direction;
  uint8_t active;
  uint8_t imu_ready;
  float yaw_delta_deg;
  float heading_integral_deg_s;
  float transition_distance_m;
} straight_controller_t;

/** Initialize a caller-owned straight-line controller state. */
void StraightController_Init(straight_controller_t *controller);

/** Reset accumulated heading and caster-transition state. */
void StraightController_Reset(straight_controller_t *controller);

/** Compute one bounded straight-line target update without changing external state. */
straight_controller_result_t StraightController_Step(straight_controller_t *controller,
                                                      const straight_controller_params_t *params,
                                                      const straight_controller_input_t *input);

#endif
