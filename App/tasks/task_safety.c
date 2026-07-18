#include "app_tasks.h"

#include "imu_calibration_orchestrator.h"
#include "robot_config.h"
#include "system_publish_snapshot_collector.h"
#include "system_publish_snapshot_service.h"
#include "command_management_service.h"
#include "platform_time.h"
#include "platform_watchdog.h"
#include "power_on_self_test_service.h"
#include "parameter_management_service.h"
#include "platform_reset_trace.h"
#include "safety_management_service.h"
#include "system_monitoring_service.h"
#include "state_estimation_service.h"

void Task_Safety(void *argument)
{
    uint32_t                                   next_wake      = PlatformTime_TaskNowMs();
    const robot_config_t                      *config         = RobotConfig_GetDefault();
    const communication_publish_model_config_t publish_config = {
        .host_timeout_ms   = config->display.rpi_timeout_ms,
        .esp12f_timeout_ms = config->command.esp12f_timeout_ms,
        .line_timeout_ms   = config->display.line_timeout_ms,
    };
    (void)argument;
    for (;;)
    {
        uint32_t                      now_ms = PlatformTime_TaskNowMs();
        uint32_t                      motor_hb;
        param_model_t                 params;
        communication_publish_model_t publish_snapshot;

        (void)ParameterManagement_GetSnapshot(&params);
        SafetyManagement_SetCurrentThresholds(params.current_observe_a,
                                              params.current_fault_a,
                                              params.current_fault_debounce_ms);
        SafetyManagement_Update();
        PostService_UpdateRuntime(now_ms);
        AppSystemPublishSnapshot_Collect(now_ms, &publish_config, &publish_snapshot);
        SystemPublishSnapshot_Publish(&publish_snapshot);
        PlatformResetTrace_UpdateControl((uint8_t)CommandManagement_GetActiveSource(now_ms),
                                         SafetyManagement_IsEmergencyStop(),
                                         SafetyManagement_IsFaultStop());
        PlatformResetTrace_TaskHeartbeat(RESET_TRACE_TASK_SAFETY, now_ms);
        motor_hb = PlatformResetTrace_GetTaskHeartbeat(RESET_TRACE_TASK_MOTOR);
        if (motor_hb != 0U && (now_ms - motor_hb) <= 200U)
        {
            PlatformWatchdog_Feed();
        }
        ImuCalibrationOrchestrator_ProcessPersistence(now_ms);
        SystemMonitoring_DelayUntil(SYSTEM_MONITORING_TASK_SAFETY,
                                    &next_wake,
                                    RobotConfig_GetDefault()->tasks[APP_TASK_SAFETY].period_ms);
    }
}
