#include "app_tasks.h"

#include "chassis_config.h"
#include "line_control_service.h"
#include "line_uart.h"
#include "platform_time.h"
#include "task_health_service.h"

void Task_Line(void *argument)
{
    uint32_t next_wake = PlatformTime_TaskNowMs();
    (void)argument;
    for (;;)
    {
        LineUart_Update();
        LineUart_RequestAnalog();
        LineControlService_Update();
        TaskHealthService_DelayUntil(TASK_HEALTH_SERVICE_LINE, &next_wake, CHASSIS_LINE_PERIOD_MS);
    }
}
