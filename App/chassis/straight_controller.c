#include "straight_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define STRAIGHT_MIN_ACTIVE_MPS 0.0001f
#define STRAIGHT_TRIM_LOW_SPEED_MPS 0.15f
#define STRAIGHT_TRIM_HIGH_SPEED_MPS 0.30f
#define STRAIGHT_CORRECTION_RATIO 0.25f
#define STRAIGHT_CORRECTION_MAX_MPS 0.075f
#define STRAIGHT_TRANSITION_DISTANCE_M 0.30f
#define STRAIGHT_PWM_SATURATED_PERMILLE 850
#define STRAIGHT_MAX_DT_S 0.20f

static float ClampFloat(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

static float InterpolateTrim(float speed_mps, float low_trim_mps, float high_trim_mps)
{
  if (speed_mps <= STRAIGHT_TRIM_LOW_SPEED_MPS)
  {
    return low_trim_mps * speed_mps / STRAIGHT_TRIM_LOW_SPEED_MPS;
  }
  if (speed_mps >= STRAIGHT_TRIM_HIGH_SPEED_MPS)
  {
    return high_trim_mps;
  }
  return low_trim_mps +
         (high_trim_mps - low_trim_mps) *
           (speed_mps - STRAIGHT_TRIM_LOW_SPEED_MPS) /
           (STRAIGHT_TRIM_HIGH_SPEED_MPS - STRAIGHT_TRIM_LOW_SPEED_MPS);
}

static uint8_t OutputIsSaturated(int16_t output_permille, uint8_t current_limited)
{
  int32_t magnitude = output_permille;
  if (magnitude < 0)
  {
    magnitude = -magnitude;
  }
  return (uint8_t)((magnitude >= STRAIGHT_PWM_SATURATED_PERMILLE) || (current_limited != 0U));
}

static uint8_t TargetIncreasesMagnitude(float base_mps, float target_mps)
{
  return (uint8_t)((base_mps * target_mps > 0.0f) && (fabsf(target_mps) > fabsf(base_mps)));
}

void StraightController_Init(straight_controller_t *controller)
{
  if (controller != NULL)
  {
    memset(controller, 0, sizeof(*controller));
  }
}

void StraightController_Reset(straight_controller_t *controller)
{
  StraightController_Init(controller);
}

straight_controller_result_t StraightController_Step(straight_controller_t *controller,
                                                      const straight_controller_params_t *params,
                                                      const straight_controller_input_t *input)
{
  straight_controller_result_t result = {0};
  float dt_s;
  float speed_mps;
  float signed_base_mps;
  float correction_limit_mps;
  float unclamped_correction_mps;
  float candidate_integral_deg_s;
  float candidate_left_mps;
  float candidate_right_mps;
  uint8_t wheel_valid;
  uint8_t left_saturated;
  uint8_t right_saturated;
  uint8_t reset_reference;
  int8_t direction;

  if ((controller == NULL) || (params == NULL) || (input == NULL))
  {
    return result;
  }

  direction = (input->requested_linear_mps > 0.0f) ? 1 : -1;
  if ((fabsf(input->requested_linear_mps) <= STRAIGHT_MIN_ACTIVE_MPS) ||
      (fabsf(input->requested_angular_rps) > STRAIGHT_MIN_ACTIVE_MPS))
  {
    StraightController_Reset(controller);
    return result;
  }

  reset_reference = (uint8_t)((controller->active == 0U) || (controller->direction != direction) ||
                              (controller->source != input->source) ||
                              (controller->generation != input->generation));
  if (reset_reference != 0U)
  {
    controller->yaw_delta_deg = 0.0f;
    controller->heading_integral_deg_s = 0.0f;
    controller->transition_distance_m = 0.0f;
    controller->imu_ready = 0U;
    controller->last_ms = input->now_ms;
  }

  dt_s = (float)(uint32_t)(input->now_ms - controller->last_ms) * 0.001f;
  dt_s = ClampFloat(dt_s, 0.0f, STRAIGHT_MAX_DT_S);
  controller->last_ms = input->now_ms;
  controller->active = 1U;
  controller->direction = direction;
  controller->source = input->source;
  controller->generation = input->generation;
  candidate_integral_deg_s = controller->heading_integral_deg_s;

  speed_mps = ClampFloat(fabsf(input->requested_linear_mps), 0.0f, params->max_speed_mps);
  signed_base_mps = (float)direction * speed_mps;
  result.active = 1U;
  result.direction = direction;

  if (direction > 0)
  {
    result.trim_correction_mps = InterpolateTrim(speed_mps,
                                                  params->trim_forward_015_mps,
                                                  params->trim_forward_030_mps);
  }
  else
  {
    result.trim_correction_mps = InterpolateTrim(speed_mps,
                                                  params->trim_reverse_015_mps,
                                                  params->trim_reverse_030_mps);
  }

  wheel_valid = (uint8_t)((input->left_speed_valid != 0U) && (input->right_speed_valid != 0U));
  if (wheel_valid != 0U)
  {
    result.wheel_correction_mps =
      params->wheel_coupling_gain * (input->actual_left_mps - input->actual_right_mps);
    controller->transition_distance_m +=
      0.5f * (fabsf(input->actual_left_mps) + fabsf(input->actual_right_mps)) * dt_s;
  }
  result.transition_distance_m = controller->transition_distance_m;
  result.in_transition = (uint8_t)(controller->transition_distance_m < STRAIGHT_TRANSITION_DISTANCE_M);

  if ((params->heading_enabled == 0U) || (input->imu_valid == 0U))
  {
    result.heading_degraded = (uint8_t)(params->heading_enabled != 0U);
    controller->yaw_delta_deg = 0.0f;
    controller->heading_integral_deg_s = 0.0f;
    controller->imu_ready = 0U;
  }
  else if (controller->imu_ready == 0U)
  {
    controller->imu_ready = 1U;
    controller->yaw_delta_deg = 0.0f;
    controller->heading_integral_deg_s = 0.0f;
  }
  else
  {
    controller->yaw_delta_deg += input->gyro_z_dps * dt_s;
    result.heading_error_deg = -controller->yaw_delta_deg;
    candidate_integral_deg_s = ClampFloat(
      controller->heading_integral_deg_s + result.heading_error_deg * dt_s,
      -params->heading_integral_limit_deg_s,
      params->heading_integral_limit_deg_s);
    result.heading_correction_mps =
      params->heading_kp * result.heading_error_deg + params->heading_ki * candidate_integral_deg_s;
  }

  correction_limit_mps = fminf(STRAIGHT_CORRECTION_MAX_MPS, STRAIGHT_CORRECTION_RATIO * speed_mps);
  unclamped_correction_mps = result.trim_correction_mps + result.wheel_correction_mps +
                             result.heading_correction_mps;
  result.total_correction_mps =
    ClampFloat(unclamped_correction_mps, -correction_limit_mps, correction_limit_mps);
  result.correction_clamped =
    (uint8_t)(fabsf(unclamped_correction_mps - result.total_correction_mps) > 0.000001f);

  left_saturated = OutputIsSaturated(input->left_output_permille, input->left_current_limited);
  right_saturated = OutputIsSaturated(input->right_output_permille, input->right_current_limited);
  if ((params->heading_enabled != 0U) && (input->imu_valid != 0U) && (controller->imu_ready != 0U) &&
      (result.correction_clamped == 0U) && (left_saturated == 0U) && (right_saturated == 0U))
  {
    controller->heading_integral_deg_s = candidate_integral_deg_s;
  }
  result.heading_integral_deg_s = controller->heading_integral_deg_s;

  candidate_left_mps = signed_base_mps - result.total_correction_mps;
  candidate_right_mps = signed_base_mps + result.total_correction_mps;
  if ((left_saturated != 0U) && TargetIncreasesMagnitude(signed_base_mps, candidate_left_mps))
  {
    candidate_right_mps += signed_base_mps - candidate_left_mps;
    candidate_left_mps = signed_base_mps;
    result.derated = 1U;
  }
  else if ((right_saturated != 0U) && TargetIncreasesMagnitude(signed_base_mps, candidate_right_mps))
  {
    candidate_left_mps += signed_base_mps - candidate_right_mps;
    candidate_right_mps = signed_base_mps;
    result.derated = 1U;
  }

  result.left_target_mps = candidate_left_mps;
  result.right_target_mps = candidate_right_mps;
  return result;
}
