#include "pid_controller.h"

void PidController_Init(pid_state_t *pid, const pid_params_t *params)
{
  if (pid == 0 || params == 0)
  {
    return;
  }
  pid->params = *params;
  PidController_Reset(pid);
  pid->initialized = 1U;
}

void PidController_Reset(pid_state_t *pid)
{
  if (pid == 0)
  {
    return;
  }
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}

void PidController_SetParams(pid_state_t *pid, const pid_params_t *params)
{
  if (pid == 0 || params == 0)
  {
    return;
  }
  pid->params = *params;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}

float PidController_StepLimited(pid_state_t *pid,
                                float target,
                                float actual,
                                float dt_s,
                                int8_t actuator_limit_direction)
{
  float error;
  float p_term;
  float i_term;
  float d_term;
  float candidate_integral;
  float candidate_output;
  float output;
  uint8_t freeze_integral = 0U;

  if (pid == 0 || pid->initialized == 0U || dt_s <= 0.0f)
  {
    return 0.0f;
  }

  error = target - actual;
  p_term = pid->params.kp * error;

  candidate_integral = pid->integral + (error * dt_s);
  if (candidate_integral > pid->params.integral_limit)
  {
    candidate_integral = pid->params.integral_limit;
  }
  else if (candidate_integral < -pid->params.integral_limit)
  {
    candidate_integral = -pid->params.integral_limit;
  }

  d_term = pid->params.kd * (error - pid->prev_error) / dt_s;
  pid->prev_error = error;

  candidate_output = p_term + (pid->params.ki * candidate_integral) + d_term;
  if ((candidate_output > pid->params.output_limit && error > 0.0f) ||
      (candidate_output < -pid->params.output_limit && error < 0.0f))
  {
    freeze_integral = 1U;
  }
  if ((actuator_limit_direction > 0 && error > 0.0f) ||
      (actuator_limit_direction < 0 && error < 0.0f))
  {
    freeze_integral = 1U;
  }
  if (freeze_integral == 0U)
  {
    pid->integral = candidate_integral;
  }
  i_term = pid->params.ki * pid->integral;

  output = p_term + i_term + d_term;
  if (output > pid->params.output_limit)
  {
    output = pid->params.output_limit;
  }
  else if (output < -pid->params.output_limit)
  {
    output = -pid->params.output_limit;
  }

  return output;
}

float PidController_Step(pid_state_t *pid, float target, float actual, float dt_s)
{
  return PidController_StepLimited(pid, target, actual, dt_s, 0);
}
