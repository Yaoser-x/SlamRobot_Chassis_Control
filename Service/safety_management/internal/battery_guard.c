#include "battery_guard.h"

void BatteryGuard_Init(battery_guard_t *guard)
{
    if (guard != 0)
    {
        *guard = (battery_guard_t){0};
    }
}

void BatteryGuard_Update(battery_guard_t                  *guard,
                         const safety_management_config_t *config,
                         float                             battery_voltage,
                         uint8_t                           sample_valid,
                         uint8_t                           critical_latched,
                         uint32_t                          now_ms,
                         battery_guard_result_t           *result)
{
    if (guard == 0 || config == 0 || result == 0)
    {
        return;
    }
    *result = (battery_guard_result_t){0};

    if (config->battery_low_monitor_enabled != 0U && sample_valid != 0U)
    {
        if (battery_voltage < config->battery_low_warn_v)
        {
            guard->warning_active = 1U;
        }
        else if (battery_voltage > config->battery_low_clear_v)
        {
            guard->warning_active = 0U;
        }
    }
    result->warning_active = guard->warning_active;

    if (critical_latched == 0U)
    {
        guard->recovery_debounce_active = 0U;
        if (sample_valid != 0U && battery_voltage < config->battery_critical_v)
        {
            if (guard->critical_debounce_active == 0U)
            {
                guard->critical_debounce_active = 1U;
                guard->critical_since_ms        = now_ms;
            }
            else if ((uint32_t)(now_ms - guard->critical_since_ms) >= config->battery_critical_debounce_ms)
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
    if (sample_valid != 0U && battery_voltage > config->battery_recover_v)
    {
        if (guard->recovery_debounce_active == 0U)
        {
            guard->recovery_debounce_active = 1U;
            guard->recovery_since_ms        = now_ms;
        }
        else if ((uint32_t)(now_ms - guard->recovery_since_ms) >= config->battery_recover_debounce_ms)
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
