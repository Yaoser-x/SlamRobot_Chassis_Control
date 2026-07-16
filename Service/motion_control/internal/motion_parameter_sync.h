#ifndef CHASSIS_PARAM_SYNC_H
#define CHASSIS_PARAM_SYNC_H

#include <stdint.h>

#include "parameter_management_service.h"
#include "motion_control_config.h"
#include "wheel_speed_pid_controller.h"
#include "motor_types.h"

typedef struct
{
    param_model_t params;
    uint32_t      generation;
    float         pid_correction_limit;
} motion_parameter_sync_t;

/** Initialize chassis runtime parameter synchronization state. */
void MotionParameterSync_Init(motion_parameter_sync_t *sync, const motion_control_config_t *config);

/** Refresh runtime parameters and PID instances when generation changes. */
uint8_t MotionParameterSync_Refresh(motion_parameter_sync_t *sync, pid_state_t pid_motor[MOTOR_ID_COUNT]);

#endif
