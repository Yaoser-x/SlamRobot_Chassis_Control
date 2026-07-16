#include "app_tasks.h"

#include "robot_config.h"
#include "communication_publish_model_service.h"
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

        (void)CommunicationPublishModel_Get(&publish_model);
        HostCommunication_Update(&publish_model);
        SystemMonitoring_DelayUntil(SYSTEM_MONITORING_TASK_HOST,
                                    &next_wake,
                                    RobotConfig_GetDefault()->tasks[APP_TASK_HOST].period_ms);
    }
}
