#include "wheel_speed_pid_controller.h"

static uint8_t PidController_Finite(float value)
{
    const float max_float = 3.402823466e+38f;
    return (value == value && value <= max_float && value >= -max_float) ? 1U : 0U;
}

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
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

void PidController_SetParams(pid_state_t *pid, const pid_params_t *params)
{
    if (pid == 0 || params == 0)
    {
        return;
    }
    pid->params     = *params;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

float PidController_StepBounded(pid_state_t *pid,
                                float        target,
                                float        actual,
                                float        dt_s,
                                int8_t       actuator_limit_direction,
                                float        output_min,
                                float        output_max)
{
    float   error;
    float   p_term;
    float   i_term;
    float   d_term;
    float   candidate_integral;
    float   candidate_output;
    float   output;
    uint8_t freeze_integral = 0U;

    if (pid == 0 || pid->initialized == 0U || PidController_Finite(target) == 0U || PidController_Finite(actual) == 0U
        || PidController_Finite(dt_s) == 0U || PidController_Finite(output_min) == 0U
        || PidController_Finite(output_max) == 0U || dt_s <= 0.0f || output_min > output_max)
    {
        PidController_Reset(pid);
        return 0.0f;
    }

    if (output_min < -pid->params.output_limit)
    {
        output_min = -pid->params.output_limit;
    }
    if (output_max > pid->params.output_limit)
    {
        output_max = pid->params.output_limit;
    }
    if (output_min > output_max)
    {
        return 0.0f;
    }

    error  = target - actual;
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

    d_term          = pid->params.kd * (error - pid->prev_error) / dt_s;
    pid->prev_error = error;

    candidate_output = p_term + (pid->params.ki * candidate_integral) + d_term;
    if ((candidate_output > output_max && error > 0.0f) || (candidate_output < output_min && error < 0.0f))
    {
        freeze_integral = 1U;
    }
    if ((actuator_limit_direction > 0 && error > 0.0f) || (actuator_limit_direction < 0 && error < 0.0f))
    {
        freeze_integral = 1U;
    }
    if (freeze_integral == 0U)
    {
        pid->integral = candidate_integral;
    }
    i_term = pid->params.ki * pid->integral;

    output = p_term + i_term + d_term;
    if (PidController_Finite(output) == 0U)
    {
        PidController_Reset(pid);
        return 0.0f;
    }
    if (output > output_max)
    {
        output = output_max;
    }
    else if (output < output_min)
    {
        output = output_min;
    }

    return output;
}

float PidController_StepLimited(pid_state_t *pid,
                                float        target,
                                float        actual,
                                float        dt_s,
                                int8_t       actuator_limit_direction)
{
    float output_limit = (pid != 0) ? pid->params.output_limit : 0.0f;

    return PidController_StepBounded(pid, target, actual, dt_s, actuator_limit_direction, -output_limit, output_limit);
}

float PidController_Step(pid_state_t *pid, float target, float actual, float dt_s)
{
    return PidController_StepLimited(pid, target, actual, dt_s, 0);
}
