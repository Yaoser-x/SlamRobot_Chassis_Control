#ifndef SYSTEM_MONITORING_SERVICE_H
#define SYSTEM_MONITORING_SERVICE_H

#include <stdint.h>

#include "system_monitoring_config.h"
#include "system_monitoring_status.h"

#ifdef __cplusplus
extern "C"
{
#endif

    uint8_t  SystemMonitoring_Init(const system_monitoring_config_t *config, uint32_t reset_reason_flags);
    uint8_t  SystemMonitoring_IsInitialized(void);
    uint32_t SystemMonitoring_NextWake(uint32_t previous_wake_ms, uint32_t now_ms, uint32_t period_ms, uint8_t *missed);
    void     SystemMonitoring_DelayUntil(system_monitoring_task_id_t task, uint32_t *next_wake_ms, uint32_t period_ms);
    void     SystemMonitoring_Heartbeat(system_monitoring_task_id_t task, uint32_t now_ms);
    void     SystemMonitoring_UpdateTimeouts(uint32_t now_ms);
    void     SystemMonitoring_ResetTaskHealth(void);
    uint32_t SystemMonitoring_GetMissedCount(system_monitoring_task_id_t task);
    uint16_t SystemMonitoring_GetTimeoutMask(void);
    uint32_t SystemMonitoring_GetTaskHealth(system_monitoring_task_health_t *health);
    uint32_t SystemMonitoring_GetStatus(system_monitoring_status_t *status);
    void     SystemMonitoring_SetModuleHealth(const system_monitoring_module_health_t *modules);
    void     SystemMonitoring_CaptureResetReason(uint32_t flags);
    uint32_t SystemMonitoring_GetResetReason(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_MONITORING_SERVICE_H */
