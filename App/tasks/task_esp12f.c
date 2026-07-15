#include "app_tasks.h"

#include "control_config.h"
#include "bsp_config.h"
#include "esp12f_flash_bridge.h"
#include "esp12f_service.h"
#include "platform_time.h"
#include "platform_reset_trace.h"
#include "task_health_service.h"

void Task_Esp12f(void *argument)
{
    uint32_t next_wake = PlatformTime_TaskNowMs();
    (void)argument;
    for (;;)
    {
        uint32_t now_ms = PlatformTime_TaskNowMs();

        PlatformResetTrace_TaskHeartbeat(RESET_TRACE_TASK_ESP, now_ms);
        Esp12fFlashBridge_Update(now_ms);
        Esp12fService_Update();
        TaskHealthService_DelayUntil(TASK_HEALTH_SERVICE_ESP, &next_wake, CHASSIS_ESP12F_PERIOD_MS);
    }
}
