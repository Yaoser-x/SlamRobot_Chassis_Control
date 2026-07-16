#ifndef SYSTEM_MONITORING_CONFIG_H
#define SYSTEM_MONITORING_CONFIG_H

#include <stdint.h>

#define SYSTEM_MONITORING_TASK_COUNT 9U

/** @brief App-derived task-health timeouts in milliseconds. */
typedef struct
{
    uint32_t task_timeout_ms[SYSTEM_MONITORING_TASK_COUNT];
} system_monitoring_config_t;

#endif
