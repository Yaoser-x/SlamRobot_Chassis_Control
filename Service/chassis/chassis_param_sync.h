#ifndef CHASSIS_PARAM_SYNC_H
#define CHASSIS_PARAM_SYNC_H

#include <stdint.h>

#include "param_service.h"
#include "pid_controller.h"

typedef struct
{
    param_model_t params;
    uint32_t      generation;
} chassis_param_sync_t;

/** Initialize chassis runtime parameter synchronization state. */
void ChassisParamSync_Init(chassis_param_sync_t *sync);

/** Refresh runtime parameters and PID instances when generation changes. */
uint8_t ChassisParamSync_Refresh(chassis_param_sync_t *sync, pid_state_t pid_motor[MOTOR_ID_COUNT]);

#endif
