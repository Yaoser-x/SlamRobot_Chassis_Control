#include "app_tasks.h"

#include "robot_config.h"
#include "oled_ui.h"
#include "platform_time.h"
#include "system_monitoring_service.h"

void Task_Oled(void *argument)
{
    uint32_t next_wake = PlatformTime_TaskNowMs();
    (void)argument;
    for (;;)
    {
        OLED_UI_Update();
        SystemMonitoring_DelayUntil(SYSTEM_MONITORING_TASK_OLED,
                                    &next_wake,
                                    RobotConfig_GetDefault()->tasks[APP_TASK_OLED].period_ms);
    }
}
