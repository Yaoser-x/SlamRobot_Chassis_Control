#include "app_tasks.h"
#include "chassis_runtime_coordinator.h"

#include "robot_config.h"
#include "motion_control_service.h"
#include "power_management_service.h"
#include "platform_time.h"
#include "platform_reset_trace.h"
#include "state_estimation_service.h"
#include "system_monitoring_service.h"

void Task_MotorControl(void *argument)
{
    uint32_t next_wake = PlatformTime_TaskNowMs();
    (void)argument;
    for (;;)
    {
        uint32_t now_ms = PlatformTime_TaskNowMs();

        PlatformResetTrace_TaskHeartbeat(RESET_TRACE_TASK_MOTOR, now_ms);
        ChassisRuntimeCoordinator_RunMotorCycle(now_ms);
        SystemMonitoring_DelayUntil(SYSTEM_MONITORING_TASK_MOTOR,
                                    &next_wake,
                                    RobotConfig_GetDefault()->tasks[APP_TASK_MOTOR].period_ms);
    }
}
