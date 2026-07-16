#include "motion_maintenance.h"

#include "motion_control_service.h"
#include "chassis_layout.h"
#include "motor_driver.h"
#include "safety_management_service.h"
#include "state_estimation_service.h"

static float maintenance_max_speed_mps;

void MotionMaintenance_Init(float max_speed_mps)
{
    maintenance_max_speed_mps = max_speed_mps;
}

static uint8_t MotionMaintenanceService_IsStationary(void)
{
    state_estimation_wheel_status_t encoder_state;
    motor_driver_state_t            motor_state;

    (void)StateEstimation_GetWheel(&encoder_state);
    MotorDriver_GetState(&motor_state);
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (motor_state.requested_pwm[i] != 0 || motor_state.applied_pwm[i] != 0 || motor_state.effective_pwm[i] != 0)
        {
            return 0U;
        }
        if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U
            && (encoder_state.speed_valid[i] == 0U || encoder_state.speed_mps[i] < -maintenance_max_speed_mps
                || encoder_state.speed_mps[i] > maintenance_max_speed_mps))
        {
            return 0U;
        }
    }
    return 1U;
}

motion_control_maintenance_result_t MotionControl_BeginMaintenance(void)
{
    if (SafetyManagement_BeginMaintenance() == 0U)
    {
        return MOTION_CONTROL_MAINTENANCE_BUSY;
    }
    if (MotionControl_IsStepActive() != 0U)
    {
        SafetyManagement_EndMaintenance();
        return MOTION_CONTROL_MAINTENANCE_BUSY;
    }

    MotionControl_CancelTestMode();
    MotionControl_EmergencyStop();
    if (MotionMaintenanceService_IsStationary() == 0U)
    {
        SafetyManagement_EndMaintenance();
        return MOTION_CONTROL_MAINTENANCE_NOT_STATIONARY;
    }
    return MOTION_CONTROL_MAINTENANCE_OK;
}

void MotionControl_EndMaintenance(void)
{
    SafetyManagement_EndMaintenance();
}
