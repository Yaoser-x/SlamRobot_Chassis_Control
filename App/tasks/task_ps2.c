#include "app_tasks.h"

#include "robot_config.h"
#include "platform_time.h"
#include "line_following_service.h"
#include "teleoperation_service.h"
#include "operator_action_router.h"
#include "platform_reset_trace.h"
#include "system_monitoring_service.h"

void Task_Ps2(void *argument)
{
    uint32_t next_wake = PlatformTime_TaskNowMs();
    (void)argument;
    for (;;)
    {
        PlatformResetTrace_TaskHeartbeat(RESET_TRACE_TASK_PS2, PlatformTime_TaskNowMs());
        teleoperation_action_t action = {0};
        action.line_tracking_enabled = LineFollowing_IsEnabled();
        Teleoperation_Update(&action);
        OperatorActionRouter_Handle(&action);
        SystemMonitoring_DelayUntil(SYSTEM_MONITORING_TASK_PS2,
                                    &next_wake,
                                    RobotConfig_GetDefault()->tasks[APP_TASK_PS2].period_ms);
    }
}
