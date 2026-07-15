#include "app_tasks.h"

#include "control_config.h"
#include "bsp_config.h"
#include "oled_ui.h"
#include "platform_time.h"
#include "task_health_service.h"

void Task_Oled(void *argument)
{
    uint32_t next_wake = PlatformTime_TaskNowMs();
    (void)argument;
    for (;;)
    {
        OLED_UI_Update();
        TaskHealthService_DelayUntil(TASK_HEALTH_SERVICE_OLED, &next_wake, OLED_TASK_PERIOD_MS);
    }
}
