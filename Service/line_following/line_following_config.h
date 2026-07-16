#ifndef LINE_FOLLOWING_CONFIG_H
#define LINE_FOLLOWING_CONFIG_H

#include <stdint.h>

/** @brief Product configuration for line availability and freshness. */
typedef struct
{
    float    angular_max_rps;
    uint32_t sensor_timeout_ms;
    uint8_t  default_enabled;
    uint8_t  detect_threshold_count;
} line_following_config_t;

#endif
