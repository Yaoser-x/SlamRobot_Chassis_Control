#include "motion_maintenance_orchestrator.h"

#include "control_mode_coordinator.h"
#include "motor_driver_snapshot_adapter.h"
#include "motion_control_maintenance.h"
#include "motion_control_service.h"
#include "robot_config.h"
#include "safety_workflow_coordinator.h"
#include "state_estimation_service.h"
#include "platform_critical.h"

static uint8_t        maintenance_active;
static control_mode_t previous_mode;
static uint8_t        diagnostic_motion_active;
static control_mode_t diagnostic_previous_mode;

static void AppMotionMaintenance_Rollback(control_mode_t mode)
{
    (void)ControlModeCoordinator_Request(mode);
    AppSafetyWorkflow_EndMaintenance();
}

static uint8_t AppMotionMaintenance_IsStationary(void)
{
    state_estimation_wheel_status_t wheel;
    app_motor_driver_snapshot_t     motor;
    float                           max_speed = RobotConfig_GetDefault()->motion.maintenance_max_speed_mps;

    if (StateEstimation_GetWheel(&wheel) == 0UL)
    {
        return 0U;
    }
    if (AppMotorDriverAdapter_GetSnapshot(&motor) == 0UL)
    {
        return 0U;
    }
    for (uint8_t index = 0U; index < APP_MOTOR_DRIVER_COUNT; ++index)
    {
        if (motor.requested_pwm[index] != 0 || motor.applied_pwm[index] != 0 || motor.effective_pwm[index] != 0)
        {
            return 0U;
        }
        if ((motor.enabled_mask & (uint8_t)(1U << index)) != 0U
            && (wheel.speed_valid[index] == 0U || wheel.speed_mps[index] < -max_speed
                || wheel.speed_mps[index] > max_speed))
        {
            return 0U;
        }
    }
    return 1U;
}

app_motion_maintenance_result_t AppMotionMaintenance_Begin(void)
{
    control_mode_snapshot_t mode;

    if (maintenance_active != 0U || diagnostic_motion_active != 0U)
    {
        return APP_MOTION_MAINTENANCE_BUSY;
    }
    (void)ControlModeCoordinator_GetSnapshot(&mode);
    if (mode.mode == CONTROL_MODE_MANUAL || AppSafetyWorkflow_BeginMaintenance() == 0U)
    {
        return APP_MOTION_MAINTENANCE_BUSY;
    }
    if (MotionControl_IsStepActive() != 0U || ControlModeCoordinator_Request(CONTROL_MODE_MAINTENANCE) == 0U)
    {
        AppMotionMaintenance_Rollback(mode.mode);
        return APP_MOTION_MAINTENANCE_BUSY;
    }

    MotionControl_CancelTestMode();
    MotionControl_EmergencyStop();
    if (AppMotionMaintenance_IsStationary() == 0U)
    {
        AppMotionMaintenance_Rollback(mode.mode);
        return APP_MOTION_MAINTENANCE_NOT_STATIONARY;
    }
    previous_mode      = mode.mode;
    maintenance_active = 1U;
    return APP_MOTION_MAINTENANCE_OK;
}

app_motion_maintenance_result_t AppMotionMaintenance_TryBeginStationary(void)
{
    control_mode_snapshot_t mode;
    platform_critical_state_t critical = PlatformCritical_Enter();

    (void)ControlModeCoordinator_GetSnapshot(&mode);
    if (maintenance_active != 0U || diagnostic_motion_active != 0U || mode.mode == CONTROL_MODE_MANUAL
        || MotionControl_IsStepActive() != 0U)
    {
        PlatformCritical_Exit(critical);
        return APP_MOTION_MAINTENANCE_BUSY;
    }
    if (AppMotionMaintenance_IsStationary() == 0U)
    {
        PlatformCritical_Exit(critical);
        return APP_MOTION_MAINTENANCE_NOT_STATIONARY;
    }
    if (AppSafetyWorkflow_BeginMaintenance() == 0U)
    {
        PlatformCritical_Exit(critical);
        return APP_MOTION_MAINTENANCE_BUSY;
    }
    if (ControlModeCoordinator_Request(CONTROL_MODE_MAINTENANCE) == 0U)
    {
        AppMotionMaintenance_Rollback(mode.mode);
        PlatformCritical_Exit(critical);
        return APP_MOTION_MAINTENANCE_BUSY;
    }
    previous_mode      = mode.mode;
    maintenance_active = 1U;
    PlatformCritical_Exit(critical);

    /* Recheck after ownership is acquired, before any Flash operation. */
    if (AppMotionMaintenance_IsStationary() == 0U)
    {
        AppMotionMaintenance_End();
        return APP_MOTION_MAINTENANCE_NOT_STATIONARY;
    }
    MotionControl_CancelTestMode();
    MotionControl_EmergencyStop();
    return APP_MOTION_MAINTENANCE_OK;
}

void AppMotionMaintenance_End(void)
{
    if (maintenance_active == 0U)
    {
        return;
    }
    maintenance_active = 0U;
    (void)ControlModeCoordinator_Request(previous_mode);
    AppSafetyWorkflow_EndMaintenance();
}

app_motion_maintenance_result_t AppMotionMaintenance_BeginDiagnosticMotion(void)
{
    control_mode_snapshot_t mode;

    if (maintenance_active != 0U || diagnostic_motion_active != 0U)
    {
        return APP_MOTION_MAINTENANCE_BUSY;
    }
    (void)ControlModeCoordinator_GetSnapshot(&mode);
    if (mode.mode == CONTROL_MODE_MANUAL || MotionControl_IsStepActive() != 0U)
    {
        return APP_MOTION_MAINTENANCE_BUSY;
    }
    MotionControl_CancelTestMode();
    if (AppMotionMaintenance_IsStationary() == 0U)
    {
        return APP_MOTION_MAINTENANCE_NOT_STATIONARY;
    }
    if (ControlModeCoordinator_Request(CONTROL_MODE_MAINTENANCE) == 0U)
    {
        return APP_MOTION_MAINTENANCE_BUSY;
    }
    diagnostic_previous_mode = mode.mode;
    diagnostic_motion_active = 1U;
    return APP_MOTION_MAINTENANCE_OK;
}

void AppMotionMaintenance_EndDiagnosticMotion(void)
{
    if (diagnostic_motion_active == 0U)
    {
        return;
    }
    diagnostic_motion_active = 0U;
    (void)ControlModeCoordinator_Request(diagnostic_previous_mode);
}
