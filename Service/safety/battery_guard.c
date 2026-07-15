#include "battery_guard.h"

#include "control_config.h"

void BatteryGuard_Init(battery_guard_t *guard)
{
    if (guard != 0)
    {
        *guard = (battery_guard_t){0};
    }
}

void BatteryGuard_Update(battery_guard_t        *guard,
                         float                   battery_voltage,
                         uint8_t                 sample_valid,
                         uint8_t                 critical_latched,
                         uint32_t                now_ms,
                         battery_guard_result_t *result)
{
    if (guard == 0 || result == 0)
    {
        return;
    }
    *result = (battery_guard_result_t){0};

    if (BATTERY_LOW_MONITOR_ENABLED != 0U && sample_valid != 0U)
    {
        if (battery_voltage < BATTERY_LOW_WARN_V)
        {
            guard->warning_active = 1U;
        }
        else if (battery_voltage > BATTERY_LOW_CLEAR_V)
        {
            guard->warning_active = 0U;
        }
    }
    result->warning_active = guard->warning_active;

    if (critical_latched == 0U)
    {
        guard->recovery_debounce_active = 0U;
        if (sample_valid != 0U && battery_voltage < BATTERY_CRITICAL_V)
        {
            if (guard->critical_debounce_active == 0U)
            {
                guard->critical_debounce_active = 1U;
                guard->critical_since_ms        = now_ms;
            }
            else if ((uint32_t)(now_ms - guard->critical_since_ms) >= BATTERY_CRITICAL_DEBOUNCE_MS)
            {
                result->latch_critical          = 1U;
                guard->critical_debounce_active = 0U;
            }
        }
        else
        {
            guard->critical_debounce_active = 0U;
        }
        return;
    }

    guard->critical_debounce_active = 0U;
    if (sample_valid != 0U && battery_voltage > BATTERY_RECOVER_V)
    {
        if (guard->recovery_debounce_active == 0U)
        {
            guard->recovery_debounce_active = 1U;
            guard->recovery_since_ms        = now_ms;
        }
        else if ((uint32_t)(now_ms - guard->recovery_since_ms) >= BATTERY_RECOVER_DEBOUNCE_MS)
        {
            result->clear_critical          = 1U;
            guard->recovery_debounce_active = 0U;
        }
    }
    else
    {
        guard->recovery_debounce_active = 0U;
    }
}
