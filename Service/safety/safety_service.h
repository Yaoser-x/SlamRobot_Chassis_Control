#ifndef SAFETY_SERVICE_H
#define SAFETY_SERVICE_H

#include <stdint.h>

#include "task_health_service.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define SYSTEM_ERROR_LOW_BATTERY           (1UL << 0)
#define SYSTEM_ERROR_M1_OVERCURRENT        (1UL << 1)
#define SYSTEM_ERROR_M2_OVERCURRENT        (1UL << 2)
#define SYSTEM_ERROR_M3_OVERCURRENT        (1UL << 3)
#define SYSTEM_ERROR_M4_OVERCURRENT        (1UL << 4)
#define SYSTEM_ERROR_LEFT_OVERCURRENT      (SYSTEM_ERROR_M1_OVERCURRENT | SYSTEM_ERROR_M2_OVERCURRENT)
#define SYSTEM_ERROR_RIGHT_OVERCURRENT     (SYSTEM_ERROR_M3_OVERCURRENT | SYSTEM_ERROR_M4_OVERCURRENT)
#define SYSTEM_ERROR_ESTOP                 (1UL << 5)
#define SYSTEM_ERROR_FAULT_STOP            (1UL << 6)
#define SYSTEM_ERROR_ENCODER_INVALID       (1UL << 7)
#define SYSTEM_ERROR_DRV_FAULT             (1UL << 8)
#define SYSTEM_ERROR_TIM_BREAK             (1UL << 9)
#define SYSTEM_ERROR_ENCODER_FEEDBACK_LOST (1UL << 17)
#define SYSTEM_ERROR_BATTERY_CRITICAL      (1UL << 18)

    typedef struct
    {
        float    battery_voltage;
        float    motor_current_a[4];
        float    left_current_a;
        float    right_current_a;
        uint32_t current_observe_over_limit_count[4];
        uint32_t current_fault_would_latch_count[4];
        uint32_t error_flags;
        uint32_t latched_error_flags;
        uint32_t task_last_heartbeat_ms[TASK_HEALTH_SERVICE_COUNT];
        uint32_t task_timeout_count[TASK_HEALTH_SERVICE_COUNT];
        uint8_t  task_timed_out[TASK_HEALTH_SERVICE_COUNT];
        uint16_t task_timeout_mask;
        uint8_t  control_mode;
        uint8_t  current_control_valid;
        uint8_t  current_control_valid_mask;
    } safety_service_snapshot_t;

    void    SafetyService_Init(void);
    void    SafetyService_Update(void);
    void    SafetyService_GetState(safety_service_snapshot_t *state);
    void    SafetyService_ClearLatchedFaults(uint32_t mask);
    uint8_t SafetyService_HasLatchedFault(void);
    void    SafetyService_LatchEncoderFeedbackFault(void);

#ifdef __cplusplus
}
#endif

#endif
