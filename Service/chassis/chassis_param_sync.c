#include "chassis_param_sync.h"

#include "control_config.h"
#include "motor_driver.h"

void ChassisParamSync_Init(chassis_param_sync_t *sync)
{
    if (sync != 0)
    {
        *sync = (chassis_param_sync_t){0};
    }
}

uint8_t ChassisParamSync_Refresh(chassis_param_sync_t *sync, pid_state_t pid_motor[MOTOR_ID_COUNT])
{
    param_model_t params;
    uint32_t      generation;

    if (sync == 0 || pid_motor == 0)
    {
        return 0U;
    }
    generation = ParamService_GetSnapshot(&params);
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
            CHASSIS_PID_CORRECTION_LIMIT,
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
