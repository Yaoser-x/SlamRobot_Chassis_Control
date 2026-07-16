#include "motion_parameter_sync.h"

#include "motor_driver.h"

void MotionParameterSync_Init(motion_parameter_sync_t *sync, const motion_control_config_t *config)
{
    if (sync != 0 && config != 0)
    {
        *sync                      = (motion_parameter_sync_t){0};
        sync->pid_correction_limit = config->pid_correction_limit;
    }
}

uint8_t MotionParameterSync_Refresh(motion_parameter_sync_t *sync, pid_state_t pid_motor[MOTOR_ID_COUNT])
{
    param_model_t params;
    uint32_t      generation;

    if (sync == 0 || pid_motor == 0)
    {
        return 0U;
    }
    generation = ParameterManagement_GetSnapshot(&params);
    if (generation == sync->generation)
    {
        return 0U;
    }

    sync->params     = params;
    sync->generation = generation;
    MotorDriver_SetDirectionConfig(params.motor_dir);
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        pid_params_t pid_params = {
            params.pid_kp[index],
            params.pid_ki[index],
            params.pid_kd[index],
            params.pid_integral_limit,
            sync->pid_correction_limit,
        };

        if (pid_motor[index].initialized == 0U)
        {
            PidController_Init(&pid_motor[index], &pid_params);
        }
        else
        {
            PidController_SetParams(&pid_motor[index], &pid_params);
        }
    }
    return 1U;
}
