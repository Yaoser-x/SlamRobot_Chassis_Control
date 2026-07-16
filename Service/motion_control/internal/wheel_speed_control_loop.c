#include "wheel_speed_control_loop.h"

static float WheelSpeedControlLoop_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int8_t WheelSpeedControlLoop_TargetSign(float value, float epsilon)
{
    if (value > epsilon)
    {
        return 1;
    }
    if (value < -epsilon)
    {
        return -1;
    }
    return 0;
}

static int16_t WheelSpeedControlLoop_Clamp(int32_t permille)
{
    if (permille > MOTOR_DRIVER_MAX_PERMILLE)
    {
        return MOTOR_DRIVER_MAX_PERMILLE;
    }
    if (permille < -MOTOR_DRIVER_MAX_PERMILLE)
    {
        return -MOTOR_DRIVER_MAX_PERMILLE;
    }
    return (int16_t)permille;
}

void WheelSpeedControlLoop_Init(wheel_speed_control_loop_t *loop, const motion_control_config_t *config)
{
    if (loop != 0 && config != 0)
    {
        *loop                       = (wheel_speed_control_loop_t){0};
        loop->correction_limit      = config->pid_correction_limit;
        loop->stop_epsilon_mps      = config->pid_stop_epsilon_mps;
        loop->direction_epsilon_mps = config->pid_direction_epsilon_mps;
    }
}

void WheelSpeedControlLoop_SetParams(wheel_speed_control_loop_t *loop, const param_model_t *params)
{
    if (loop == 0 || params == 0)
    {
        return;
    }
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        pid_params_t pid_params = {
            params->pid_kp[index],
            params->pid_ki[index],
            params->pid_kd[index],
            params->pid_integral_limit,
            loop->correction_limit,
        };
        if (loop->pid_motor[index].initialized == 0U)
        {
            PidController_Init(&loop->pid_motor[index], &pid_params);
        }
        else
        {
            PidController_SetParams(&loop->pid_motor[index], &pid_params);
        }
    }
}

void WheelSpeedControlLoop_Reset(wheel_speed_control_loop_t *loop)
{
    if (loop == 0)
    {
        return;
    }
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        PidController_Reset(&loop->pid_motor[index]);
        loop->last_target_mps[index] = 0.0f;
    }
}

void WheelSpeedControlLoop_ResetMotor(wheel_speed_control_loop_t *loop, motor_id_t motor)
{
    if (loop == 0 || (uint32_t)motor >= MOTOR_ID_COUNT)
    {
        return;
    }
    PidController_Reset(&loop->pid_motor[motor]);
    loop->last_target_mps[motor] = 0.0f;
}

void WheelSpeedControlLoop_ResetTargets(wheel_speed_control_loop_t *loop)
{
    if (loop == 0)
    {
        return;
    }
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        loop->last_target_mps[index] = 0.0f;
    }
}

wheel_speed_control_loop_result_t WheelSpeedControlLoop_StepMotor(wheel_speed_control_loop_t *loop,
                                                                  motor_id_t                  motor,
                                                                  float                       target_mps,
                                                                  float                       actual_mps,
                                                                  uint8_t                     speed_valid,
                                                                  float                       dt_s,
                                                                  int8_t                      actuator_limit_direction,
                                                                  int16_t                     base_permille,
                                                                  motor_driver_phase_t        phase)
{
    wheel_speed_control_loop_result_t result = {.permille = base_permille};
    int8_t                            last_sign;
    int8_t                            target_sign;
    float                             pid_out;

    if (loop == 0 || (uint32_t)motor >= MOTOR_ID_COUNT)
    {
        return result;
    }
    if (phase == MOTOR_DRIVER_PHASE_RAMP_DOWN || phase == MOTOR_DRIVER_PHASE_REVERSE_BRAKE
        || phase == MOTOR_DRIVER_PHASE_PH_SETTLE)
    {
        return result;
    }
    if (WheelSpeedControlLoop_Abs(target_mps) <= loop->stop_epsilon_mps || speed_valid == 0U)
    {
        PidController_Reset(&loop->pid_motor[motor]);
        loop->last_target_mps[motor] = target_mps;
        result.permille              = 0;
        return result;
    }

    last_sign   = WheelSpeedControlLoop_TargetSign(loop->last_target_mps[motor], loop->direction_epsilon_mps);
    target_sign = WheelSpeedControlLoop_TargetSign(target_mps, loop->direction_epsilon_mps);
    if (last_sign != 0 && target_sign != 0 && last_sign != target_sign)
    {
        PidController_Reset(&loop->pid_motor[motor]);
    }
    loop->last_target_mps[motor] = target_mps;
    result.error_mps             = target_mps - actual_mps;
    pid_out                      = PidController_StepBounded(&loop->pid_motor[motor],
                                        target_mps,
                                        actual_mps,
                                        dt_s,
                                        actuator_limit_direction,
                                        (float)(-MOTOR_DRIVER_MAX_PERMILLE - base_permille),
                                        (float)(MOTOR_DRIVER_MAX_PERMILLE - base_permille));
    result.pid_active            = 1U;
    result.permille              = WheelSpeedControlLoop_Clamp((int32_t)base_permille + (int32_t)pid_out);
    return result;
}
