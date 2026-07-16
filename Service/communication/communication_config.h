#ifndef COMMUNICATION_CONFIG_H
#define COMMUNICATION_CONFIG_H

#include <stdint.h>

/** @brief Product configuration for unchanged telemetry publication periods. */
typedef struct
{
    uint32_t host_status_period_ms;
    uint32_t host_imu_status_period_ms;
    uint32_t host_diagnostic_period_ms;
    uint32_t esp12f_status_period_ms;
} communication_config_t;

#endif
