#include "app_tasks.h"

#include "chassis_config.h"
#include "imu_calibration_service.h"
#include "imu_service.h"
#include "platform_task_event.h"
#include "platform_time.h"
#include "task_health_service.h"

void Task_Imu(void *argument)
{
    (void)argument;
    for (;;)
    {
        uint32_t now_ms;

        (void)PlatformTaskEvent_Wait(PLATFORM_TASK_EVENT_IMU_DRDY, CHASSIS_IMU_PERIOD_MS);
        (void)ImuService_RunCycle();
        now_ms = PlatformTime_TaskNowMs();
        TaskHealthService_Heartbeat(TASK_HEALTH_SERVICE_IMU, now_ms);
        ImuCalibrationService_ProcessSample(now_ms);
    }
}
