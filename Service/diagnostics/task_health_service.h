#ifndef TASK_HEALTH_SERVICE_H
#define TASK_HEALTH_SERVICE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        TASK_HEALTH_SERVICE_SAFETY = 0,
        TASK_HEALTH_SERVICE_MOTOR,
        TASK_HEALTH_SERVICE_RPI,
        TASK_HEALTH_SERVICE_IMU,
        TASK_HEALTH_SERVICE_LINE,
        TASK_HEALTH_SERVICE_ESP,
        TASK_HEALTH_SERVICE_PS2,
        TASK_HEALTH_SERVICE_LED,
        TASK_HEALTH_SERVICE_OLED,
        TASK_HEALTH_SERVICE_COUNT
    } task_health_service_id_t;

    typedef struct
    {
        uint32_t last_heartbeat_ms[TASK_HEALTH_SERVICE_COUNT];
        uint32_t timeout_count[TASK_HEALTH_SERVICE_COUNT];
        uint8_t  timed_out[TASK_HEALTH_SERVICE_COUNT];
    } chassis_task_health_t;

    uint32_t
         TaskHealthService_NextWake(uint32_t previous_wake_ms, uint32_t now_ms, uint32_t period_ms, uint8_t *missed);
    void TaskHealthService_DelayUntil(task_health_service_id_t task, uint32_t *next_wake_ms, uint32_t period_ms);
    uint32_t TaskHealthService_GetMissedCount(task_health_service_id_t task);
    void     TaskHealthService_Heartbeat(task_health_service_id_t task, uint32_t now_ms);
    void     TaskHealthService_UpdateTimeouts(uint32_t now_ms);
    void     TaskHealthService_GetHealth(chassis_task_health_t *health);
    uint16_t TaskHealthService_GetTimeoutMask(void);
    void     TaskHealthService_Reset(void);

#ifdef __cplusplus
}
#endif

#endif
