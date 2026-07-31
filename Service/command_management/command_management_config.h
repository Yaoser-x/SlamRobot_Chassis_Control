#ifndef COMMAND_MANAGEMENT_CONFIG_H
#define COMMAND_MANAGEMENT_CONFIG_H

#include <stdint.h>

/** @brief Frozen per-source command timeout configuration. */
typedef struct
{
    uint32_t host_timeout_ms;
    uint32_t ps2_timeout_ms;
    uint32_t esp12f_timeout_ms;
    uint32_t line_timeout_ms;
    uint32_t debug_timeout_ms;
    uint32_t remote_max_lifetime_ms;
} command_management_config_t;

#endif
