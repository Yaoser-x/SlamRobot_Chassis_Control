#include "app_tasks.h"

#include "robot_config.h"
#include "system_publish_snapshot_service.h"
#include "esp12f_flash_bridge.h"
#include "wireless_communication_service.h"
#include "platform_time.h"
#include "platform_reset_trace.h"
#include "system_monitoring_service.h"

void Task_Esp12f(void *argument)
{
    uint32_t next_wake = PlatformTime_TaskNowMs();
    (void)argument;
    for (;;)
    {
        uint32_t                      now_ms = PlatformTime_TaskNowMs();
        communication_publish_model_t publish_model;

        PlatformResetTrace_TaskHeartbeat(RESET_TRACE_TASK_ESP, now_ms);
        Esp12fFlashBridge_Update(now_ms);
        (void)CommunicationPublishModel_Get(&publish_model);
        WirelessCommunication_Update(&publish_model);
        SystemMonitoring_DelayUntil(SYSTEM_MONITORING_TASK_ESP,
                                    &next_wake,
                                    RobotConfig_GetDefault()->tasks[APP_TASK_ESP12F].period_ms);
    }
}
