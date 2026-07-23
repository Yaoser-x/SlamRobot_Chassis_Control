#include "app_tasks.h"

#include "remote_operation_orchestrator.h"
#include "robot_config.h"
#include "system_publish_snapshot_service.h"
#include "platform_time.h"
#include "system_monitoring_service.h"
#include "host_communication_service.h"

void Task_RpiComm(void *argument)
{
    uint32_t next_wake = PlatformTime_TaskNowMs();
    (void)argument;
    for (;;)
    {
        communication_publish_model_t publish_model;

        (void)SystemPublishSnapshot_Get(&publish_model);
        HostCommunication_Update(&publish_model);
        for (;;)
        {
            communication_operation_request_t request;
            uint32_t                          now_ms = PlatformTime_TaskNowMs();
            uint32_t                          detail_mask;
            communication_operation_stage_t   result;

            if (HostCommunication_TakeOperation(now_ms, &request) == 0U)
            {
                break;
            }
            result = RemoteOperationOrchestrator_Dispatch(&request, &detail_mask);
            HostCommunication_CompleteOperation(&request, result, detail_mask, PlatformTime_TaskNowMs());
        }
        SystemMonitoring_DelayUntil(SYSTEM_MONITORING_TASK_HOST,
                                    &next_wake,
                                    RobotConfig_GetDefault()->tasks[APP_TASK_HOST].period_ms);
    }
}
