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

float PidController_Step(pid_state_t *pid, float target, float actual, float dt_s)
{
  float error;
  float p_term;
  float i_term;
  float d_term;
  float output;

  if (pid == 0 || pid->initialized == 0U || dt_s <= 0.0f)
  {
    return 0.0f;
  }

  error = target - actual;
  p_term = pid->params.kp * error;

  pid->integral += error * dt_s;
  if (pid->integral > pid->params.integral_limit)
  {
    pid->integral = pid->params.integral_limit;
  }
  else if (pid->integral < -pid->params.integral_limit)
  {
    pid->integral = -pid->params.integral_limit;
  }
  i_term = pid->params.ki * pid->integral;

  d_term = pid->params.kd * (error - pid->prev_error) / dt_s;
  pid->prev_error = error;

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
