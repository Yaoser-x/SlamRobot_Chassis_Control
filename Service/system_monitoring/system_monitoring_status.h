#ifndef SYSTEM_MONITORING_STATUS_H
#define SYSTEM_MONITORING_STATUS_H

#include <stdint.h>

#include "system_monitoring_config.h"

typedef enum
{
    SYSTEM_MONITORING_TASK_SAFETY = 0,
    SYSTEM_MONITORING_TASK_MOTOR,
    SYSTEM_MONITORING_TASK_HOST,
    SYSTEM_MONITORING_TASK_IMU,
    SYSTEM_MONITORING_TASK_LINE,
    SYSTEM_MONITORING_TASK_ESP,
    SYSTEM_MONITORING_TASK_PS2,
    SYSTEM_MONITORING_TASK_LED,
    SYSTEM_MONITORING_TASK_OLED,
    SYSTEM_MONITORING_TASK_ID_COUNT = SYSTEM_MONITORING_TASK_COUNT
} system_monitoring_task_id_t;

typedef struct
{
    uint32_t last_heartbeat_ms[SYSTEM_MONITORING_TASK_COUNT];
    uint32_t timeout_count[SYSTEM_MONITORING_TASK_COUNT];
    uint32_t missed_count[SYSTEM_MONITORING_TASK_COUNT];
    uint8_t  timed_out[SYSTEM_MONITORING_TASK_COUNT];
} system_monitoring_task_health_t;

typedef struct
{
    uint8_t imu_online;
    uint8_t encoder_online;
    uint8_t motor_online;
    uint8_t adc_online;
    uint8_t host_online;
    uint8_t esp12f_online;
    uint8_t line_online;
    uint8_t ps2_online;
} system_monitoring_module_health_t;

/** @brief System-only health facts; no chassis business snapshot is stored here. */
typedef struct
{
    system_monitoring_task_health_t   task_health;
    system_monitoring_module_health_t modules;
    uint32_t                          reset_reason_flags;
    uint32_t                          generation;
} system_monitoring_status_t;

#endif /* SYSTEM_MONITORING_STATUS_H */
