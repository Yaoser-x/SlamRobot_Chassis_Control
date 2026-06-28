#include "chassis_control.h"

#include "adc_monitor.h"
#include "chassis_config.h"
#include "chassis_layout.h"
#include "chassis_math.h"
#include "chassis_output_slew.h"
#include "control_manager.h"
#include "encoder_driver.h"
#include "main.h"
#include "motor_driver.h"
#include "pid_controller.h"

static chassis_control_state_t chassis_state;
static uint8_t open_loop_test_enabled;
static uint8_t raw_input_test_enabled;
static int16_t open_loop_side[2];
static int16_t raw_forward[MOTOR_ID_COUNT];
static int16_t raw_reverse[MOTOR_ID_COUNT];
static int16_t output_slew_permille[MOTOR_ID_COUNT];
static float ramped_linear_x;
static float ramped_angular_z;
static float last_pid_target_mps[MOTOR_ID_COUNT];
static float last_requested_mps[MOTOR_ID_COUNT];
static uint8_t feedback_loss_count[MOTOR_ID_COUNT];
static pid_state_t pid_motor[MOTOR_ID_COUNT];

static const pid_params_t pid_params[MOTOR_ID_COUNT] = {
  { CHASSIS_PID_KP_M1, CHASSIS_PID_KI_M1, CHASSIS_PID_KD_M1, CHASSIS_PID_INTEGRAL_LIMIT, CHASSIS_PID_CORRECTION_LIMIT },
  { CHASSIS_PID_KP_M2, CHASSIS_PID_KI_M2, CHASSIS_PID_KD_M2, CHASSIS_PID_INTEGRAL_LIMIT, CHASSIS_PID_CORRECTION_LIMIT },
  { CHASSIS_PID_KP_M3, CHASSIS_PID_KI_M3, CHASSIS_PID_KD_M3, CHASSIS_PID_INTEGRAL_LIMIT, CHASSIS_PID_CORRECTION_LIMIT },
  { CHASSIS_PID_KP_M4, CHASSIS_PID_KI_M4, CHASSIS_PID_KD_M4, CHASSIS_PID_INTEGRAL_LIMIT, CHASSIS_PID_CORRECTION_LIMIT },
};

static int16_t ChassisControl_ClampPermille(int32_t permille)
{
  if (permille > CHASSIS_PWM_MAX_PERMILLE)
  {
    return CHASSIS_PWM_MAX_PERMILLE;
  }
  if (permille < -CHASSIS_PWM_MAX_PERMILLE)
  {
    return -CHASSIS_PWM_MAX_PERMILLE;
  }
  return (int16_t)permille;
}

static float ChassisControl_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static int8_t ChassisControl_TargetSign(float value)
{
  if (value > CHASSIS_PID_DIRECTION_EPSILON_MPS)
  {
    return 1;
  }
  if (value < -CHASSIS_PID_DIRECTION_EPSILON_MPS)
  {
    return -1;
  }
  return 0;
}

void ChassisControl_ResolveSideTargets(float linear_x, float angular_z, float *left_mps, float *right_mps)
{
  ChassisMath_ResolveDifferentialTargets(linear_x,
                                         angular_z,
                                         CHASSIS_WHEEL_BASE_M,
                                         left_mps,
                                         right_mps);
}

static float ChassisControl_RampToward(float current, float target, float step)
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

static void ChassisControl_ResetRamps(void)
{
  ramped_linear_x = 0.0f;
  ramped_angular_z = 0.0f;
}

static void ChassisControl_ResetPidTargets(void)
{
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    last_pid_target_mps[i] = 0.0f;
    last_requested_mps[i] = 0.0f;
    feedback_loss_count[i] = 0U;
    chassis_state.motor_feedback_lost[i] = 0U;
  }
  chassis_state.left_feedback_lost = 0U;
  chassis_state.right_feedback_lost = 0U;
}

static void ChassisControl_ResetOutputSlew(void)
{
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    output_slew_permille[i] = 0;
  }
}

static int16_t ChassisControl_StepOutputSlew(motor_id_t motor, int16_t target)
{
  output_slew_permille[motor] = ChassisOutputSlew_Step(output_slew_permille[motor],
                                                       target,
                                                       CHASSIS_OUTPUT_SLEW_STEP_PER_CYCLE);
  return output_slew_permille[motor];
}

static int16_t ChassisControl_MpsToPermille(float target_mps)
{
  int32_t permille;

  if (CHASSIS_OPENLOOP_FULL_MPS <= 0.0f)
  {
    return 0;
  }
  if (target_mps > CHASSIS_OPENLOOP_FULL_MPS)
  {
    target_mps = CHASSIS_OPENLOOP_FULL_MPS;
  }
  else if (target_mps < -CHASSIS_OPENLOOP_FULL_MPS)
  {
    target_mps = -CHASSIS_OPENLOOP_FULL_MPS;
  }

  permille = (int32_t)((target_mps / CHASSIS_OPENLOOP_FULL_MPS) * (float)CHASSIS_PWM_MAX_PERMILLE);
  return ChassisControl_ClampPermille(permille);
}

static int16_t ChassisControl_ApplyCurrentLimit(int16_t permille, float current_a, uint8_t *limited)
{
  int32_t scaled;

  if (limited != 0)
  {
    *limited = 0U;
  }
  if (MOTOR_CURRENT_LIMIT_A <= 0.0f || current_a <= MOTOR_CURRENT_LIMIT_A || permille == 0)
  {
    return permille;
  }

  scaled = (int32_t)((float)permille * (MOTOR_CURRENT_LIMIT_A / current_a));
  if (limited != 0)
  {
    *limited = 1U;
  }
  return ChassisControl_ClampPermille(scaled);
}

static void ChassisControl_SetMotorOutput(motor_id_t motor, int16_t permille)
{
  adc_monitor_state_t adc_state;
  int16_t applied;

  if (ChassisLayout_MotorEnabled(motor) == 0U)
  {
    chassis_state.motor_current_limited[motor] = 0U;
    chassis_state.motor_output_permille[motor] = 0;
    output_slew_permille[motor] = 0;
    MotorDriver_SetPermille(motor, 0);
    return;
  }

  permille = ChassisControl_ClampPermille(permille);
  applied = ChassisControl_StepOutputSlew(motor, permille);

  AdcMonitor_GetState(&adc_state);
  chassis_state.motor_current_limited[motor] = 0U;
  if (adc_state.current_valid != 0U)
  {
    applied = ChassisControl_ApplyCurrentLimit(applied,
                                                adc_state.current_a[motor],
                                                &chassis_state.motor_current_limited[motor]);
  }
  output_slew_permille[motor] = applied;
  chassis_state.motor_output_permille[motor] = applied;
  MotorDriver_SetPermille(motor, applied);
}

static float ChassisControl_SelectSideValue(motor_id_t motor, float left_value, float right_value)
{
  return (ChassisLayout_MotorSide(motor) == MOTOR_SIDE_LEFT) ? left_value : right_value;
}

static uint8_t ChassisControl_AnyActiveMotorOutput(void)
{
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U &&
        chassis_state.motor_output_permille[i] != 0)
    {
      return 1U;
    }
  }
  return 0U;
}

static void ChassisControl_SyncSideState(void)
{
  uint8_t left_count = 0U;
  uint8_t right_count = 0U;
  float left_target_sum = 0.0f;
  float right_target_sum = 0.0f;
  float left_requested_sum = 0.0f;
  float right_requested_sum = 0.0f;
  float left_actual_sum = 0.0f;
  float right_actual_sum = 0.0f;
  float left_error_sum = 0.0f;
  float right_error_sum = 0.0f;
  int32_t left_output_sum = 0;
  int32_t right_output_sum = 0;

  chassis_state.left_speed_valid = (ChassisLayout_SideMotorCount(MOTOR_SIDE_LEFT) != 0U) ? 1U : 0U;
  chassis_state.right_speed_valid = (ChassisLayout_SideMotorCount(MOTOR_SIDE_RIGHT) != 0U) ? 1U : 0U;
  chassis_state.left_pid_active = 0U;
  chassis_state.right_pid_active = 0U;
  chassis_state.left_feedback_lost = 0U;
  chassis_state.right_feedback_lost = 0U;
  chassis_state.left_current_limited = 0U;
  chassis_state.right_current_limited = 0U;

  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    motor_id_t motor = (motor_id_t)i;
    if (ChassisLayout_MotorEnabled(motor) == 0U)
    {
      continue;
    }

    if (ChassisLayout_MotorSide(motor) == MOTOR_SIDE_LEFT)
    {
      left_count++;
      left_target_sum += chassis_state.motor_target_mps[i];
      left_requested_sum += chassis_state.motor_requested_mps[i];
      left_actual_sum += chassis_state.motor_actual_mps[i];
      left_error_sum += chassis_state.motor_error_mps[i];
      left_output_sum += chassis_state.motor_output_permille[i];
      if (chassis_state.motor_speed_valid[i] == 0U)
      {
        chassis_state.left_speed_valid = 0U;
      }
      chassis_state.left_pid_active |= chassis_state.motor_pid_active[i];
      chassis_state.left_feedback_lost |= chassis_state.motor_feedback_lost[i];
      chassis_state.left_current_limited |= chassis_state.motor_current_limited[i];
    }
    else
    {
      right_count++;
      right_target_sum += chassis_state.motor_target_mps[i];
      right_requested_sum += chassis_state.motor_requested_mps[i];
      right_actual_sum += chassis_state.motor_actual_mps[i];
      right_error_sum += chassis_state.motor_error_mps[i];
      right_output_sum += chassis_state.motor_output_permille[i];
      if (chassis_state.motor_speed_valid[i] == 0U)
      {
        chassis_state.right_speed_valid = 0U;
      }
      chassis_state.right_pid_active |= chassis_state.motor_pid_active[i];
      chassis_state.right_feedback_lost |= chassis_state.motor_feedback_lost[i];
      chassis_state.right_current_limited |= chassis_state.motor_current_limited[i];
    }
  }

  chassis_state.left_target_mps = (left_count != 0U) ? (left_target_sum / (float)left_count) : 0.0f;
  chassis_state.right_target_mps = (right_count != 0U) ? (right_target_sum / (float)right_count) : 0.0f;
  chassis_state.left_requested_mps = (left_count != 0U) ? (left_requested_sum / (float)left_count) : 0.0f;
  chassis_state.right_requested_mps = (right_count != 0U) ? (right_requested_sum / (float)right_count) : 0.0f;
  chassis_state.left_actual_mps = (left_count != 0U) ? (left_actual_sum / (float)left_count) : 0.0f;
  chassis_state.right_actual_mps = (right_count != 0U) ? (right_actual_sum / (float)right_count) : 0.0f;
  chassis_state.left_error_mps = (left_count != 0U) ? (left_error_sum / (float)left_count) : 0.0f;
  chassis_state.right_error_mps = (right_count != 0U) ? (right_error_sum / (float)right_count) : 0.0f;
  chassis_state.left_output_permille = (left_count != 0U) ? (int16_t)(left_output_sum / (int32_t)left_count) : 0;
  chassis_state.right_output_permille = (right_count != 0U) ? (int16_t)(right_output_sum / (int32_t)right_count) : 0;
}

static void ChassisControl_SetSideTargets(float left_mps, float right_mps, uint8_t requested)
{
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    motor_id_t motor = (motor_id_t)i;
    float target = 0.0f;

    if (ChassisLayout_MotorEnabled(motor) != 0U)
    {
      target = ChassisControl_SelectSideValue(motor, left_mps, right_mps);
    }
    chassis_state.motor_target_mps[i] = target;
    if (requested != 0U)
    {
      chassis_state.motor_requested_mps[i] = target;
    }
  }
}

static uint8_t ChassisControl_CheckFeedbackUsable(motor_id_t motor, float target_mps, float actual_mps, uint8_t encoder_valid)
{
  chassis_state.motor_feedback_lost[motor] = 0U;
  if (encoder_valid == 0U)
  {
    feedback_loss_count[motor] = CHASSIS_PID_FEEDBACK_LOSS_COUNT;
    chassis_state.motor_feedback_lost[motor] = 1U;
    return 0U;
  }
  if (ChassisControl_AbsFloat(target_mps) < CHASSIS_PID_FEEDBACK_MIN_TARGET_MPS ||
      ChassisControl_AbsFloat(actual_mps) >= CHASSIS_PID_FEEDBACK_MIN_SPEED_MPS)
  {
    feedback_loss_count[motor] = 0U;
    return 1U;
  }
  if (feedback_loss_count[motor] < CHASSIS_PID_FEEDBACK_LOSS_COUNT)
  {
    feedback_loss_count[motor]++;
  }
  if (feedback_loss_count[motor] >= CHASSIS_PID_FEEDBACK_LOSS_COUNT)
  {
    chassis_state.motor_feedback_lost[motor] = 1U;
    return 0U;
  }
  return 1U;
}

static int16_t ChassisControl_StepMotorPid(motor_id_t motor, float target_mps, float actual_mps, uint8_t speed_valid)
{
  int8_t last_sign;
  int8_t target_sign;
  float dt_s = (float)CHASSIS_CONTROL_PERIOD_MS / 1000.0f;
  float pid_out;

  chassis_state.motor_pid_active[motor] = 0U;
  chassis_state.motor_error_mps[motor] = 0.0f;

  if (ChassisControl_AbsFloat(target_mps) <= CHASSIS_PID_STOP_EPSILON_MPS || speed_valid == 0U)
  {
    PidController_Reset(&pid_motor[motor]);
    last_pid_target_mps[motor] = target_mps;
    return 0;
  }

  last_sign = ChassisControl_TargetSign(last_pid_target_mps[motor]);
  target_sign = ChassisControl_TargetSign(target_mps);
  if (last_sign != 0 && target_sign != 0 && last_sign != target_sign)
  {
    PidController_Reset(&pid_motor[motor]);
  }
  last_pid_target_mps[motor] = target_mps;

  chassis_state.motor_error_mps[motor] = target_mps - actual_mps;
  pid_out = PidController_Step(&pid_motor[motor], target_mps, actual_mps, dt_s);
  chassis_state.motor_pid_active[motor] = 1U;
  return ChassisControl_ClampPermille((int32_t)ChassisControl_MpsToPermille(target_mps) + (int32_t)pid_out);
}

static void ChassisControl_StopOutput(void)
{
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    chassis_state.motor_target_mps[i] = 0.0f;
    chassis_state.motor_requested_mps[i] = 0.0f;
    chassis_state.motor_error_mps[i] = 0.0f;
    chassis_state.motor_pid_active[i] = 0U;
    chassis_state.motor_feedback_lost[i] = 0U;
    ChassisControl_SetMotorOutput((motor_id_t)i, 0);
  }
  ChassisControl_ResetRamps();
  ChassisControl_ResetPidTargets();
  chassis_state.output_enabled = ChassisControl_AnyActiveMotorOutput();
  ChassisControl_SyncSideState();
}

void ChassisControl_Init(void)
{
  MotorDriver_Init();
  ControlManager_Init();
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    PidController_Init(&pid_motor[i], &pid_params[i]);
  }
  chassis_state = (chassis_control_state_t){0};
  open_loop_test_enabled = 0U;
  raw_input_test_enabled = 0U;
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    raw_forward[i] = 0;
    raw_reverse[i] = 0;
  }
  open_loop_side[MOTOR_SIDE_LEFT] = 0;
  open_loop_side[MOTOR_SIDE_RIGHT] = 0;
  ChassisControl_ResetRamps();
  ChassisControl_ResetPidTargets();
  ChassisControl_ResetOutputSlew();
  MotorDriver_StopAll(MOTOR_STOP_LOW_SIDE_BRAKE);
}

void ChassisControl_Step(uint32_t now_ms)
{
  chassis_cmd_t cmd;
  encoder_state_t encoder_state;
  uint8_t valid_cmd;

  MotorDriver_UpdateFaults();
  if (MotorDriver_HasFault() != 0U)
  {
    ControlManager_SetFaultStop(1U);
  }
  valid_cmd = ControlManager_GetCommand(&cmd, now_ms);
  EncoderDriver_GetState(&encoder_state);
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    chassis_state.motor_actual_mps[i] = encoder_state.speed_mps[i];
    chassis_state.motor_speed_valid[i] = encoder_state.speed_valid[i];
  }

  if (ControlManager_IsEmergencyStop() != 0U || ControlManager_IsFaultStop() != 0U)
  {
    ChassisControl_EmergencyStop();
    return;
  }

  if (open_loop_test_enabled != 0U)
  {
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
      PidController_Reset(&pid_motor[i]);
      chassis_state.motor_pid_active[i] = 0U;
      chassis_state.motor_feedback_lost[i] = 0U;
      chassis_state.motor_error_mps[i] = 0.0f;
      if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U)
      {
        ChassisControl_SetMotorOutput((motor_id_t)i,
                                      ChassisControl_SelectSideValue((motor_id_t)i,
                                                                    (float)open_loop_side[MOTOR_SIDE_LEFT],
                                                                    (float)open_loop_side[MOTOR_SIDE_RIGHT]));
      }
      else
      {
        ChassisControl_SetMotorOutput((motor_id_t)i, 0);
      }
    }
    chassis_state.output_enabled = ChassisControl_AnyActiveMotorOutput();
    ChassisControl_ResetRamps();
    ChassisControl_SetSideTargets(0.0f, 0.0f, 1U);
    ChassisControl_SyncSideState();
    return;
  }

  if (raw_input_test_enabled != 0U)
  {
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
      PidController_Reset(&pid_motor[i]);
      if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U)
      {
        int16_t target = ChassisControl_ClampPermille((int32_t)raw_forward[i] - (int32_t)raw_reverse[i]);
        int16_t applied = ChassisControl_StepOutputSlew((motor_id_t)i, target);
        if (applied >= 0)
        {
          MotorDriver_SetInputPermille((motor_id_t)i, applied, 0);
        }
        else
        {
          MotorDriver_SetInputPermille((motor_id_t)i, 0, (int16_t)-applied);
        }
        chassis_state.motor_output_permille[i] = applied;
      }
      else
      {
        MotorDriver_SetInputPermille((motor_id_t)i, 0, 0);
        output_slew_permille[i] = 0;
        chassis_state.motor_output_permille[i] = 0;
      }
      chassis_state.motor_pid_active[i] = 0U;
      chassis_state.motor_feedback_lost[i] = 0U;
      chassis_state.motor_current_limited[i] = 0U;
      chassis_state.motor_error_mps[i] = 0.0f;
    }
    ChassisControl_ResetRamps();
    ChassisControl_SetSideTargets(0.0f, 0.0f, 1U);
    chassis_state.output_enabled = ChassisControl_AnyActiveMotorOutput();
    ChassisControl_SyncSideState();
    return;
  }

  if (valid_cmd != 0U)
  {
    float req_left;
    float req_right;
    float ramp_left;
    float ramp_right;
    float linear_step = CHASSIS_SPEED_RAMP_MPS2 * ((float)CHASSIS_CONTROL_PERIOD_MS / 1000.0f);
    float angular_step = CHASSIS_ANGULAR_RAMP_RPS2 * ((float)CHASSIS_CONTROL_PERIOD_MS / 1000.0f);

    if (ChassisLayout_HasBothSides() == 0U)
    {
      ControlManager_ClearCommand();
      ChassisControl_StopOutput();
      return;
    }

    ChassisControl_ResolveSideTargets(cmd.linear_x, cmd.angular_z, &req_left, &req_right);
    ChassisControl_SetSideTargets(req_left, req_right, 1U);

    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
      if (chassis_state.motor_requested_mps[i] != last_requested_mps[i])
      {
        PidController_Reset(&pid_motor[i]);
        last_requested_mps[i] = chassis_state.motor_requested_mps[i];
      }
    }

    ramped_linear_x = ChassisControl_RampToward(ramped_linear_x, cmd.linear_x, linear_step);
    ramped_angular_z = ChassisControl_RampToward(ramped_angular_z, cmd.angular_z, angular_step);
    ChassisControl_ResolveSideTargets(ramped_linear_x, ramped_angular_z, &ramp_left, &ramp_right);
    ChassisControl_SetSideTargets(ramp_left, ramp_right, 0U);

    if (req_left == 0.0f && req_right == 0.0f)
    {
      for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
      {
        PidController_Reset(&pid_motor[i]);
      }
      ChassisControl_ResetPidTargets();
      for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
      {
        ChassisControl_SetMotorOutput((motor_id_t)i, 0);
      }
      chassis_state.output_enabled = ChassisControl_AnyActiveMotorOutput();
      ChassisControl_SyncSideState();
      return;
    }

    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
      int16_t permille;
      if (ChassisLayout_MotorEnabled((motor_id_t)i) == 0U)
      {
        PidController_Reset(&pid_motor[i]);
        chassis_state.motor_pid_active[i] = 0U;
        chassis_state.motor_feedback_lost[i] = 0U;
        chassis_state.motor_error_mps[i] = 0.0f;
        ChassisControl_SetMotorOutput((motor_id_t)i, 0);
        continue;
      }
      if (CHASSIS_PID_ENABLED != 0U)
      {
        uint8_t feedback_usable = ChassisControl_CheckFeedbackUsable((motor_id_t)i,
                                                                      chassis_state.motor_target_mps[i],
                                                                      chassis_state.motor_actual_mps[i],
                                                                      encoder_state.speed_valid[i]);
        chassis_state.motor_speed_valid[i] = feedback_usable;
        permille = ChassisControl_StepMotorPid((motor_id_t)i,
                                               chassis_state.motor_target_mps[i],
                                               chassis_state.motor_actual_mps[i],
                                               feedback_usable);
      }
      else
      {
        chassis_state.motor_pid_active[i] = 0U;
        chassis_state.motor_feedback_lost[i] = 0U;
        chassis_state.motor_error_mps[i] = 0.0f;
        permille = ChassisControl_MpsToPermille(chassis_state.motor_target_mps[i]);
      }
      ChassisControl_SetMotorOutput((motor_id_t)i, permille);
    }
    chassis_state.output_enabled = 1U;
    ChassisControl_SyncSideState();
  }
  else
  {
    ChassisControl_StopOutput();
  }
}

void ChassisControl_EmergencyStop(void)
{
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    PidController_Reset(&pid_motor[i]);
  }
  open_loop_test_enabled = 0U;
  raw_input_test_enabled = 0U;
  ChassisControl_ResetRamps();
  ChassisControl_ResetPidTargets();
  ChassisControl_ResetOutputSlew();
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    chassis_state.motor_target_mps[i] = 0.0f;
    chassis_state.motor_requested_mps[i] = 0.0f;
    chassis_state.motor_error_mps[i] = 0.0f;
    chassis_state.motor_output_permille[i] = 0;
    chassis_state.motor_current_limited[i] = 0U;
    chassis_state.motor_pid_active[i] = 0U;
    chassis_state.motor_feedback_lost[i] = 0U;
  }
  chassis_state.output_enabled = 0U;
  MotorDriver_StopAll(MOTOR_STOP_LOW_SIDE_BRAKE);
  ChassisControl_SyncSideState();
}

void ChassisControl_OpenLoopTest(int16_t left_permille, int16_t right_permille)
{
  open_loop_side[MOTOR_SIDE_LEFT] = ChassisControl_ClampPermille(left_permille);
  open_loop_side[MOTOR_SIDE_RIGHT] = ChassisControl_ClampPermille(right_permille);
  open_loop_test_enabled = ((left_permille != 0) || (right_permille != 0)) ? 1U : 0U;
  raw_input_test_enabled = 0U;
}

void ChassisControl_RawInputTest(int16_t left_forward_permille,
                                 int16_t left_reverse_permille,
                                 int16_t right_forward_permille,
                                 int16_t right_reverse_permille)
{
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U &&
        ChassisLayout_MotorSide((motor_id_t)i) == MOTOR_SIDE_LEFT)
    {
      raw_forward[i] = ChassisControl_ClampPermille(left_forward_permille);
      raw_reverse[i] = ChassisControl_ClampPermille(left_reverse_permille);
    }
    else if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U)
    {
      raw_forward[i] = ChassisControl_ClampPermille(right_forward_permille);
      raw_reverse[i] = ChassisControl_ClampPermille(right_reverse_permille);
    }
    else
    {
      raw_forward[i] = 0;
      raw_reverse[i] = 0;
    }
    if (raw_forward[i] < 0)
    {
      raw_forward[i] = 0;
    }
    if (raw_reverse[i] < 0)
    {
      raw_reverse[i] = 0;
    }
  }
  raw_input_test_enabled = 0U;
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    if (raw_forward[i] != 0 || raw_reverse[i] != 0)
    {
      raw_input_test_enabled = 1U;
    }
  }
  open_loop_test_enabled = 0U;
}

void ChassisControl_RawMotorInputTest(motor_id_t motor, int16_t forward_permille, int16_t reverse_permille)
{
  if ((uint32_t)motor >= MOTOR_ID_COUNT)
  {
    return;
  }
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    raw_forward[i] = 0;
    raw_reverse[i] = 0;
  }
  if (ChassisLayout_MotorEnabled(motor) == 0U)
  {
    raw_input_test_enabled = 0U;
    open_loop_test_enabled = 0U;
    return;
  }
  raw_forward[motor] = ChassisControl_ClampPermille(forward_permille);
  raw_reverse[motor] = ChassisControl_ClampPermille(reverse_permille);
  if (raw_forward[motor] < 0)
  {
    raw_forward[motor] = 0;
  }
  if (raw_reverse[motor] < 0)
  {
    raw_reverse[motor] = 0;
  }
  raw_input_test_enabled = ((raw_forward[motor] != 0) || (raw_reverse[motor] != 0)) ? 1U : 0U;
  open_loop_test_enabled = 0U;
}

void ChassisControl_GetState(chassis_control_state_t *state)
{
  uint32_t primask;

  if (state != 0)
  {
    primask = __get_PRIMASK();
    __disable_irq();
    *state = chassis_state;
    __set_PRIMASK(primask);
  }
}
