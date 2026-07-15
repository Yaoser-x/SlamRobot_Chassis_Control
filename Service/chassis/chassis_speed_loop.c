#include "chassis_speed_loop.h"

#include "bsp_config.h"
#include "control_config.h"

static float ChassisSpeedLoop_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int8_t ChassisSpeedLoop_TargetSign(float value)
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

static int16_t ChassisSpeedLoop_Clamp(int32_t permille)
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

void ChassisSpeedLoop_Init(chassis_speed_loop_t *loop)
{
    if (loop != 0)
    {
        *loop = (chassis_speed_loop_t){0};
    }
}

void ChassisSpeedLoop_SetParams(chassis_speed_loop_t *loop, const param_model_t *params)
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
            CHASSIS_PID_CORRECTION_LIMIT,
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

void ChassisSpeedLoop_Reset(chassis_speed_loop_t *loop)
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

void ChassisSpeedLoop_ResetMotor(chassis_speed_loop_t *loop, motor_id_t motor)
{
    if (loop == 0 || (uint32_t)motor >= MOTOR_ID_COUNT)
    {
        return;
    }
    PidController_Reset(&loop->pid_motor[motor]);
    loop->last_target_mps[motor] = 0.0f;
}

void ChassisSpeedLoop_ResetTargets(chassis_speed_loop_t *loop)
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

chassis_speed_loop_result_t ChassisSpeedLoop_StepMotor(chassis_speed_loop_t *loop,
                                                       motor_id_t            motor,
                                                       float                 target_mps,
                                                       float                 actual_mps,
                                                       uint8_t               speed_valid,
                                                       float                 dt_s,
                                                       int8_t                actuator_limit_direction,
                                                       int16_t               base_permille,
                                                       motor_driver_phase_t  phase)
{
    chassis_speed_loop_result_t result = {.permille = base_permille};
    int8_t                      last_sign;
    int8_t                      target_sign;
    float                       pid_out;

    if (loop == 0 || (uint32_t)motor >= MOTOR_ID_COUNT)
    {
        return result;
    }
    if (phase == MOTOR_DRIVER_PHASE_RAMP_DOWN || phase == MOTOR_DRIVER_PHASE_REVERSE_BRAKE
        || phase == MOTOR_DRIVER_PHASE_PH_SETTLE)
    {
        return result;
    }
    if (ChassisSpeedLoop_Abs(target_mps) <= CHASSIS_PID_STOP_EPSILON_MPS || speed_valid == 0U)
    {
        PidController_Reset(&loop->pid_motor[motor]);
        loop->last_target_mps[motor] = target_mps;
        result.permille              = 0;
        return result;
    }

    last_sign   = ChassisSpeedLoop_TargetSign(loop->last_target_mps[motor]);
    target_sign = ChassisSpeedLoop_TargetSign(target_mps);
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
                                        (float)(-CHASSIS_PWM_MAX_PERMILLE - base_permille),
                                        (float)(CHASSIS_PWM_MAX_PERMILLE - base_permille));
    result.pid_active            = 1U;
    result.permille              = ChassisSpeedLoop_Clamp((int32_t)base_permille + (int32_t)pid_out);
    return result;
}
