#include "chassis_maintenance.h"

#include "chassis_config.h"
#include "chassis_control.h"
#include "chassis_layout.h"
#include "control_manager.h"
#include "encoder_driver.h"
#include "motor_driver.h"

static uint8_t ChassisMaintenance_IsStationary(void)
{
    encoder_state_t      encoder_state;
    motor_driver_state_t motor_state;

    EncoderDriver_GetState(&encoder_state);
    MotorDriver_GetState(&motor_state);
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (motor_state.requested_pwm[i] != 0 || motor_state.applied_pwm[i] != 0 || motor_state.effective_pwm[i] != 0)
        {
            return 0U;
        }
        if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U
            && (encoder_state.speed_valid[i] == 0U || encoder_state.speed_mps[i] < -CHASSIS_MAINTENANCE_MAX_SPEED_MPS
                || encoder_state.speed_mps[i] > CHASSIS_MAINTENANCE_MAX_SPEED_MPS))
        {
            return 0U;
        }
    }
    return 1U;
}

chassis_maintenance_result_t ChassisMaintenance_Begin(void)
{
    if (ControlManager_BeginMaintenance() == 0U)
    {
        return CHASSIS_MAINTENANCE_BUSY;
    }
    if (ChassisControl_IsStepActive() != 0U)
    {
        ControlManager_EndMaintenance();
        return CHASSIS_MAINTENANCE_BUSY;
    }

    ChassisControl_CancelTestMode();
    ChassisControl_EmergencyStop();
    MotorDriver_StopAll(MOTOR_STOP_LOW_SIDE_BRAKE);
    if (ChassisMaintenance_IsStationary() == 0U)
    {
        ControlManager_EndMaintenance();
        return CHASSIS_MAINTENANCE_NOT_STATIONARY;
    }
    return CHASSIS_MAINTENANCE_OK;
}

void ChassisMaintenance_End(void)
{
    ControlManager_EndMaintenance();
}
