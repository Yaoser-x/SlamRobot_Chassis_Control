#include "app_tasks.h"

#include "robot_config.h"
#include "app_line_sensor_calibration.h"
#include "line_following_service.h"
#include "line_sensor_driver.h"
#include "platform_time.h"
#include "system_monitoring_service.h"

void Task_Line(void *argument)
{
    uint32_t next_wake = PlatformTime_TaskNowMs();
    (void)argument;
    for (;;)
    {
        LineSensorDriver_Update();
        LineSensorDriver_RequestAnalog();
        AppLineSensorCalibration_ProcessRequest();
        LineFollowing_Update();
        SystemMonitoring_DelayUntil(SYSTEM_MONITORING_TASK_LINE,
                                    &next_wake,
                                    RobotConfig_GetDefault()->tasks[APP_TASK_LINE].period_ms);
    }
}
