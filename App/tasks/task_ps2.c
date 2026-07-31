#include "app_tasks.h"

#include "robot_config.h"
#include "platform_time.h"
#include "command_management_service.h"
#include "control_mode_coordinator.h"
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
        teleoperation_status_t status = {0};
        control_mode_event_t   mode_event;
        uint8_t                line_tracking_enabled = LineFollowing_IsEnabled();

        Teleoperation_Update(line_tracking_enabled, &action);
        (void)Teleoperation_GetStatus(&status);
        mode_event = ControlModeCoordinator_UpdatePs2(&status, PlatformTime_TaskNowMs());
        if (mode_event == CONTROL_MODE_EVENT_ENTERED_MANUAL)
        {
            if (LineFollowing_IsEnabled() != 0U)
            {
                (void)LineFollowing_Enable(0U);
            }
            Teleoperation_OnManualModeEntered(CommandManagement_GetMotionRevokeGeneration());
        }
        else if (mode_event == CONTROL_MODE_EVENT_RESTORED_LINE)
        {
            /* Restore only the mode. A later explicit line request must provide the active intent. */
            (void)LineFollowing_Enable(0U);
        }
        else if (mode_event == CONTROL_MODE_EVENT_PS2_DISCONNECTED)
        {
            (void)LineFollowing_Enable(0U);
        }
        OperatorActionRouter_Handle(&action);
        SystemMonitoring_DelayUntil(SYSTEM_MONITORING_TASK_PS2,
                                    &next_wake,
                                    RobotConfig_GetDefault()->tasks[APP_TASK_PS2].period_ms);
    }
}
