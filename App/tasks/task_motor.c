#include "app_tasks.h"

#include "chassis_config.h"
#include "chassis_service.h"
#include "current_sensor_service.h"
#include "encoder_service.h"
#include "platform_time.h"
#include "platform_reset_trace.h"
#include "task_health_service.h"

void Task_MotorControl(void *argument)
{
    uint32_t next_wake = PlatformTime_TaskNowMs();
    (void)argument;
    for (;;)
    {
        uint32_t now_ms = PlatformTime_TaskNowMs();

        PlatformResetTrace_TaskHeartbeat(RESET_TRACE_TASK_MOTOR, now_ms);
        EncoderService_Update(now_ms);
        CurrentSensorService_UpdateStationary();
        ChassisService_Step(now_ms);
        TaskHealthService_DelayUntil(TASK_HEALTH_SERVICE_MOTOR, &next_wake, CHASSIS_CONTROL_PERIOD_MS);
    }
}
