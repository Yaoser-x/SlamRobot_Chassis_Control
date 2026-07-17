#include "app_tasks.h"

#include "robot_config.h"
#include "status_led_adapter.h"
#include "platform_time.h"
#include "system_monitoring_service.h"

void Task_Led(void *argument)
{
    uint32_t next_wake = PlatformTime_TaskNowMs();
    (void)argument;
    for (;;)
    {
        const uint32_t period_ms = RobotConfig_GetDefault()->tasks[APP_TASK_LED].period_ms;
        StatusLedAdapter_TaskStep(period_ms);
        SystemMonitoring_DelayUntil(SYSTEM_MONITORING_TASK_LED, &next_wake, period_ms);
    }
}
