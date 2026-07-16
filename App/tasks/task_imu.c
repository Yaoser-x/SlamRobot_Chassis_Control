#include "app_tasks.h"

#include "robot_config.h"
#include "state_estimation_service.h"
#include "platform_task_event.h"
#include "platform_time.h"
#include "state_estimation_service.h"
#include "system_monitoring_service.h"

void Task_Imu(void *argument)
{
    (void)argument;
    for (;;)
    {
        uint32_t now_ms;

        (void)PlatformTaskEvent_Wait(PLATFORM_TASK_EVENT_IMU_DRDY,
                                     RobotConfig_GetDefault()->tasks[APP_TASK_IMU].period_ms);
        (void)StateEstimation_RunImuCycle();
        now_ms = PlatformTime_TaskNowMs();
        SystemMonitoring_Heartbeat(SYSTEM_MONITORING_TASK_IMU, now_ms);
        StateEstimation_ServiceCalibrationCoordinator(now_ms);
    }
}
