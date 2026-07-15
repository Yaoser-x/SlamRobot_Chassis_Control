#include "app_tasks.h"

#include "control_config.h"
#include "bsp_config.h"
#include "platform_time.h"
#include "ps2_control_service.h"
#include "platform_reset_trace.h"
#include "task_health_service.h"

void Task_Ps2(void *argument)
{
    uint32_t next_wake = PlatformTime_TaskNowMs();
    (void)argument;
    for (;;)
    {
        PlatformResetTrace_TaskHeartbeat(RESET_TRACE_TASK_PS2, PlatformTime_TaskNowMs());
        Ps2ControlService_Update();
        TaskHealthService_DelayUntil(TASK_HEALTH_SERVICE_PS2, &next_wake, CHASSIS_PS2_PERIOD_MS);
    }
}
