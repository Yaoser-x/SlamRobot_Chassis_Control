#include "straight_controller.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static void check(int ok, const char *msg)
{
  if (!ok)
  {
    fprintf(stderr, "FAIL: %s\n", msg);
    exit(1);
  }
}

static void check_near(float actual, float expected, float tolerance, const char *msg)
{
  check(fabsf(actual - expected) <= tolerance, msg);
}

static straight_controller_params_t DefaultParams(void)
{
  straight_controller_params_t params = {0};
  params.trim_forward_015_mps = 0.010f;
  params.trim_forward_030_mps = 0.040f;
  params.trim_reverse_015_mps = -0.020f;
  params.trim_reverse_030_mps = -0.050f;
  params.wheel_coupling_gain = 0.5f;
  params.heading_kp = 0.01f;
  params.heading_ki = 0.002f;
  params.heading_integral_limit_deg_s = 10.0f;
  params.max_speed_mps = 0.30f;
  params.heading_enabled = 1U;
  return params;
}

static straight_controller_input_t DefaultInput(void)
{
  straight_controller_input_t input = {0};
  input.now_ms = 1000U;
  input.source = 2U;
  input.generation = 7U;
  input.requested_linear_mps = 0.15f;
  input.actual_left_mps = 0.15f;
  input.actual_right_mps = 0.15f;
  input.left_speed_valid = 1U;
  input.right_speed_valid = 1U;
  input.imu_valid = 1U;
  return input;
}

static void TestDirectionalTrim(void)
{
  straight_controller_t controller;
  straight_controller_params_t params = DefaultParams();
  straight_controller_input_t input = DefaultInput();
  straight_controller_result_t result;

  StraightController_Init(&controller);
  result = StraightController_Step(&controller, &params, &input);
  check_near(result.trim_correction_mps, 0.010f, 0.0001f, "forward 0.15 trim");
  input.now_ms += 20U;
  input.requested_linear_mps = 0.225f;
  result = StraightController_Step(&controller, &params, &input);
  check_near(result.trim_correction_mps, 0.025f, 0.0001f, "forward trim interpolation");

  input.now_ms += 20U;
  input.requested_linear_mps = -0.30f;
  input.actual_left_mps = -0.30f;
  input.actual_right_mps = -0.30f;
  result = StraightController_Step(&controller, &params, &input);
  check_near(result.trim_correction_mps, -0.050f, 0.0001f, "reverse trim table is independent");
  check(result.left_target_mps > result.right_target_mps, "negative trim has correct reverse differential");
}

static void TestGyroPiAndReset(void)
{
  straight_controller_t controller;
  straight_controller_params_t params = DefaultParams();
  straight_controller_input_t input = DefaultInput();
  straight_controller_result_t result;

  params.trim_forward_015_mps = 0.0f;
  params.wheel_coupling_gain = 0.0f;
  StraightController_Init(&controller);
  (void)StraightController_Step(&controller, &params, &input);
  input.now_ms += 100U;
  input.gyro_z_dps = 10.0f;
  result = StraightController_Step(&controller, &params, &input);
  check(result.heading_error_deg < -0.9f, "positive gyro produces negative heading error");
  check(result.heading_correction_mps < 0.0f, "gyro correction follows chassis angular convention");
  check(result.heading_integral_deg_s < 0.0f, "PI integrates heading error");

  input.now_ms += 100U;
  input.requested_linear_mps = 0.0f;
  result = StraightController_Step(&controller, &params, &input);
  check(result.active == 0U, "stop disables straight controller");
  check_near(result.heading_integral_deg_s, 0.0f, 0.0001f, "stop resets integral");

  input.now_ms += 100U;
  input.requested_linear_mps = 0.15f;
  input.generation++;
  input.gyro_z_dps = 10.0f;
  result = StraightController_Step(&controller, &params, &input);
  check_near(result.heading_error_deg, 0.0f, 0.0001f, "generation change establishes a fresh gyro reference");
  input.now_ms += 100U;
  input.source++;
  result = StraightController_Step(&controller, &params, &input);
  check_near(result.heading_error_deg, 0.0f, 0.0001f, "source change establishes a fresh gyro reference");
}

static void TestDegradeRecoveryAndTickWrap(void)
{
  straight_controller_t controller;
  straight_controller_params_t params = DefaultParams();
  straight_controller_input_t input = DefaultInput();
  straight_controller_result_t result;

  params.trim_forward_015_mps = 0.0f;
  params.wheel_coupling_gain = 0.0f;
  input.now_ms = UINT32_MAX - 10U;
  StraightController_Init(&controller);
  (void)StraightController_Step(&controller, &params, &input);
  input.now_ms = 9U;
  input.gyro_z_dps = 5.0f;
  result = StraightController_Step(&controller, &params, &input);
  check_near(result.heading_error_deg, -0.1f, 0.001f, "unsigned tick delta survives wrap");

  input.now_ms += 20U;
  input.imu_valid = 0U;
  result = StraightController_Step(&controller, &params, &input);
  check(result.heading_degraded != 0U, "invalid IMU degrades in same cycle");
  check_near(result.heading_correction_mps, 0.0f, 0.0001f, "degraded mode removes PI correction");

  input.now_ms += 20U;
  input.imu_valid = 1U;
  input.gyro_z_dps = 20.0f;
  result = StraightController_Step(&controller, &params, &input);
  check_near(result.heading_correction_mps, 0.0f, 0.0001f, "IMU recovery cannot create an output step");
}

static void TestClampAntiWindupAndSaturationAllocation(void)
{
  straight_controller_t controller;
  straight_controller_params_t params = DefaultParams();
  straight_controller_input_t input = DefaultInput();
  straight_controller_result_t result;
  float integral;

  params.trim_forward_015_mps = 0.10f;
  params.wheel_coupling_gain = 0.0f;
  params.heading_kp = 0.1f;
  params.heading_ki = 0.02f;
  StraightController_Init(&controller);
  (void)StraightController_Step(&controller, &params, &input);
  input.now_ms += 100U;
  input.gyro_z_dps = -100.0f;
  result = StraightController_Step(&controller, &params, &input);
  check_near(result.total_correction_mps, 0.0375f, 0.0001f, "correction clamps to 25 percent of base speed");
  check(result.correction_clamped != 0U, "clamp is observable");
  integral = result.heading_integral_deg_s;
  input.now_ms += 100U;
  result = StraightController_Step(&controller, &params, &input);
  check_near(result.heading_integral_deg_s, integral, 0.0001f, "clamped PI freezes integral");

  params.trim_forward_015_mps = 0.020f;
  params.heading_enabled = 0U;
  StraightController_Init(&controller);
  input = DefaultInput();
  input.right_output_permille = 900;
  result = StraightController_Step(&controller, &params, &input);
  check_near(result.right_target_mps, 0.15f, 0.0001f, "saturated side is not asked to speed up");
  check(result.left_target_mps < 0.15f, "other side supplies the full differential by slowing");
  check(result.derated != 0U, "saturation derating is observable");

  params.trim_forward_015_mps = 0.0f;
  params.trim_reverse_015_mps = 0.020f;
  StraightController_Init(&controller);
  input = DefaultInput();
  input.requested_linear_mps = -0.15f;
  input.actual_left_mps = -0.15f;
  input.actual_right_mps = -0.15f;
  input.left_output_permille = -900;
  result = StraightController_Step(&controller, &params, &input);
  check_near(result.left_target_mps, -0.15f, 0.0001f,
             "reverse saturated side is not asked for more magnitude");
  check(result.right_target_mps > -0.15f,
        "reverse correction is converted to slowing the available side");

  params.trim_reverse_015_mps = 0.0f;
  params.heading_enabled = 1U;
  params.heading_kp = 0.001f;
  params.heading_ki = 0.01f;
  StraightController_Init(&controller);
  input = DefaultInput();
  (void)StraightController_Step(&controller, &params, &input);
  input.now_ms += 100U;
  input.gyro_z_dps = 1.0f;
  input.left_current_limited = 1U;
  result = StraightController_Step(&controller, &params, &input);
  check_near(result.heading_integral_deg_s, 0.0f, 0.0001f,
             "current limiting freezes heading integral");
}

static void TestCasterTransitionDistance(void)
{
  straight_controller_t controller;
  straight_controller_params_t params = DefaultParams();
  straight_controller_input_t input = DefaultInput();
  straight_controller_result_t result;
  unsigned i;

  params.trim_forward_015_mps = 0.0f;
  params.wheel_coupling_gain = 0.0f;
  params.heading_enabled = 0U;
  StraightController_Init(&controller);
  (void)StraightController_Step(&controller, &params, &input);
  for (i = 0U; i < 99U; ++i)
  {
    input.now_ms += 20U;
    result = StraightController_Step(&controller, &params, &input);
  }
  check(result.in_transition != 0U, "first 0.30m is tagged as caster transition");
  input.now_ms += 40U;
  result = StraightController_Step(&controller, &params, &input);
  check(result.transition_distance_m >= 0.30f, "transition distance integrates wheel travel");
  check(result.in_transition == 0U, "control remains active after transition segment");
}

static void TestReverseFeedbackAndPiReset(void)
{
  straight_controller_t controller;
  straight_controller_params_t params = DefaultParams();
  straight_controller_input_t input = DefaultInput();
  straight_controller_result_t result;

  params.trim_forward_015_mps = 0.0f;
  params.trim_reverse_015_mps = 0.0f;
  StraightController_Init(&controller);
  input.requested_linear_mps = -0.15f;
  input.actual_left_mps = -0.17f;
  input.actual_right_mps = -0.13f;
  (void)StraightController_Step(&controller, &params, &input);
  input.now_ms += 100U;
  input.gyro_z_dps = 10.0f;
  result = StraightController_Step(&controller, &params, &input);
  check(result.wheel_correction_mps < 0.0f, "reverse wheel coupling slows the faster-magnitude left side");
  check(result.heading_correction_mps < 0.0f, "reverse gyro PI keeps the chassis angular convention");
  check(result.heading_integral_deg_s < 0.0f, "reverse run accumulates a nonzero PI integral");

  input.now_ms += 100U;
  input.requested_linear_mps = 0.15f;
  input.actual_left_mps = 0.15f;
  input.actual_right_mps = 0.15f;
  result = StraightController_Step(&controller, &params, &input);
  check_near(result.heading_error_deg, 0.0f, 0.0001f, "direction change resets gyro delta");
  check_near(result.heading_integral_deg_s, 0.0f, 0.0001f, "direction change clears existing PI integral");
}

int main(void)
{
  TestDirectionalTrim();
  TestGyroPiAndReset();
  TestDegradeRecoveryAndTickWrap();
  TestClampAntiWindupAndSaturationAllocation();
  TestCasterTransitionDistance();
  TestReverseFeedbackAndPiReset();
  puts("PASS: straight controller");
  return 0;
}
