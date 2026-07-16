#ifndef POWER_MANAGEMENT_CONFIG_H
#define POWER_MANAGEMENT_CONFIG_H

#include <stdint.h>

/** @brief Product configuration consumed by Power Management. */
typedef struct
{
    float current_zero_max_speed_mps;
    /** App scheduler period used to derive ADC sample-rate diagnostics. */
    uint32_t update_period_ms;
} power_management_config_t;

#endif
