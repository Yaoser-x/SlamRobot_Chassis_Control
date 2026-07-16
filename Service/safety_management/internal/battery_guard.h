#ifndef BATTERY_GUARD_H
#define BATTERY_GUARD_H

#include <stdint.h>

#include "safety_management_config.h"

typedef struct
{
    uint8_t  warning_active;
    uint8_t  critical_debounce_active;
    uint8_t  recovery_debounce_active;
    uint32_t critical_since_ms;
    uint32_t recovery_since_ms;
} battery_guard_t;

typedef struct
{
    uint8_t warning_active;
    uint8_t latch_critical;
    uint8_t clear_critical;
} battery_guard_result_t;

void BatteryGuard_Init(battery_guard_t *guard);
void BatteryGuard_Update(battery_guard_t                  *guard,
                         const safety_management_config_t *config,
                         float                             battery_voltage,
                         uint8_t                           sample_valid,
                         uint8_t                           critical_latched,
                         uint32_t                          now_ms,
                         battery_guard_result_t           *result);

#endif /* BATTERY_GUARD_H */
