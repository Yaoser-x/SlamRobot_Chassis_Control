#include "app_tasks.h"

#include "control_config.h"
#include "bsp_config.h"
#include "platform_time.h"
#include "task_health_service.h"
#include "upper_uart_service.h"

void Task_RpiComm(void *argument)
{
    uint32_t next_wake = PlatformTime_TaskNowMs();
    (void)argument;
    for (;;)
    {
        UpperUartService_Update();
        TaskHealthService_DelayUntil(TASK_HEALTH_SERVICE_RPI, &next_wake, UPPER_UART_TASK_PERIOD_MS);
    }
}
