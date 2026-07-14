#include "app_tasks.h"

#include "chassis_config.h"
#include "control_service.h"
#include "imu_calibration_service.h"
#include "platform_time.h"
#include "platform_watchdog.h"
#include "post_service.h"
#include "platform_reset_trace.h"
#include "safety_service.h"
#include "task_health_service.h"

void Task_Safety(void *argument)
{
    uint32_t next_wake = PlatformTime_TaskNowMs();
    (void)argument;
    for (;;)
    {
        uint32_t now_ms = PlatformTime_TaskNowMs();
        uint32_t motor_hb;

        SafetyService_Update();
        PostService_UpdateRuntime(now_ms);
        PlatformResetTrace_UpdateControl(ControlService_GetActiveSource(),
                                         ControlService_IsEmergencyStop(),
                                         ControlService_IsFaultStop());
        PlatformResetTrace_TaskHeartbeat(RESET_TRACE_TASK_SAFETY, now_ms);
        motor_hb = PlatformResetTrace_GetTaskHeartbeat(RESET_TRACE_TASK_MOTOR);
        if (motor_hb != 0U && (now_ms - motor_hb) <= 200U)
        {
            PlatformWatchdog_Feed();
        }
        ImuCalibrationService_ProcessPersistence(now_ms);
        TaskHealthService_DelayUntil(TASK_HEALTH_SERVICE_SAFETY, &next_wake, CHASSIS_ADC_PERIOD_MS);
    }
}
