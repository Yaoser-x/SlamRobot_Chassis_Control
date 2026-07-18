#include "app_tasks.h"

#include "line_calibration_orchestrator.h"
#include "robot_config.h"
#include "status_led_adapter.h"
#include "platform_time.h"
#include "system_monitoring_service.h"

void Task_Led(void *argument)
{
    uint32_t next_wake = PlatformTime_TaskNowMs();
    (void)argument;
    for (;;)
    {
        const uint32_t                   period_ms = RobotConfig_GetDefault()->tasks[APP_TASK_LED].period_ms;
        app_line_calibration_led_event_t event;

        if (LineCalibrationOrchestrator_TakeLedEvent(&event) != 0U)
        {
            switch (event)
            {
                case APP_LINE_CALIBRATION_LED_RUNNING:
                    StatusLedAdapter_SetMode(STATUS_LED_ADAPTER_CAL_RUNNING);
                    break;
                case APP_LINE_CALIBRATION_LED_SURFACE_READY:
                    StatusLedAdapter_SetMode(STATUS_LED_ADAPTER_CAL_OK);
                    break;
                case APP_LINE_CALIBRATION_LED_APPLIED:
                    StatusLedAdapter_SetMode(STATUS_LED_ADAPTER_CAL_APPLIED);
                    break;
                case APP_LINE_CALIBRATION_LED_FAILED:
                    StatusLedAdapter_SetMode(STATUS_LED_ADAPTER_CAL_FAILED);
                    break;
                case APP_LINE_CALIBRATION_LED_NONE:
                default:
                    break;
            }
        }
        StatusLedAdapter_TaskStep(period_ms);
        SystemMonitoring_DelayUntil(SYSTEM_MONITORING_TASK_LED, &next_wake, period_ms);
    }
}
