#ifndef SAFETY_MANAGEMENT_STATUS_H
#define SAFETY_MANAGEMENT_STATUS_H

#include <stdint.h>

#include "system_monitoring_status.h"

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
    uint32_t task_last_heartbeat_ms[SYSTEM_MONITORING_TASK_COUNT];
    uint32_t task_timeout_count[SYSTEM_MONITORING_TASK_COUNT];
    uint8_t  task_timed_out[SYSTEM_MONITORING_TASK_COUNT];
    uint16_t task_timeout_mask;
    uint8_t  control_mode;
    uint8_t  current_control_valid;
    uint8_t  current_control_valid_mask;
    uint8_t  motor_fault_mask;
    uint8_t  emergency_stop;
    uint8_t  fault_stop;
    uint8_t  maintenance_lock;
    uint8_t  motion_allowed;
    uint32_t generation;
} safety_management_status_t;

#endif /* SAFETY_MANAGEMENT_STATUS_H */
