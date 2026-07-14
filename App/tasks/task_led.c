#include "app_tasks.h"

#include "chassis_config.h"
#include "led_status.h"
#include "platform_time.h"
#include "task_health_service.h"

void Task_Led(void *argument)
{
    uint32_t next_wake = PlatformTime_TaskNowMs();
    (void)argument;
    for (;;)
    {
        LedStatus_TaskStep(CHASSIS_LED_PERIOD_MS);
        TaskHealthService_DelayUntil(TASK_HEALTH_SERVICE_LED, &next_wake, CHASSIS_LED_PERIOD_MS);
    }
}
