#include "chassis_maintenance_service.h"

#include "control_config.h"
#include "chassis_service.h"
#include "chassis_layout.h"
#include "control_service.h"
#include "encoder_driver.h"
#include "motor_driver.h"

static uint8_t ChassisMaintenanceService_IsStationary(void)
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

chassis_maintenance_service_result_t ChassisMaintenanceService_Begin(void)
{
    if (ControlService_BeginMaintenance() == 0U)
    {
        return CHASSIS_MAINTENANCE_SERVICE_BUSY;
    }
    if (ChassisService_IsStepActive() != 0U)
    {
        ControlService_EndMaintenance();
        return CHASSIS_MAINTENANCE_SERVICE_BUSY;
    }

    ChassisService_CancelTestMode();
    ChassisService_EmergencyStop();
    MotorDriver_StopAll(MOTOR_STOP_LOW_SIDE_BRAKE);
    if (ChassisMaintenanceService_IsStationary() == 0U)
    {
        ControlService_EndMaintenance();
        return CHASSIS_MAINTENANCE_SERVICE_NOT_STATIONARY;
    }
    return CHASSIS_MAINTENANCE_SERVICE_OK;
}

void ChassisMaintenanceService_End(void)
{
    ControlService_EndMaintenance();
}
