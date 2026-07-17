#include "safety_management_config.h"

uint8_t SafetyManagement_ValidateConfig(const safety_management_config_t *config)
{
    if (config == 0 || config->battery_low_warn_v <= config->battery_critical_v
        || config->battery_low_clear_v <= config->battery_low_warn_v
        || config->battery_recover_v <= config->battery_critical_v || config->battery_critical_debounce_ms == 0UL
        || config->battery_recover_debounce_ms == 0UL || config->update_period_ms == 0UL
        || config->overcurrent_startup_blank_ms == 0UL || config->overcurrent_startup_rearm_ms == 0UL
        || config->current_fault_debounce_ms == 0U || config->battery_low_monitor_enabled > 1U
        || config->overcurrent_fault_enabled > 1U)
    {
        return 0U;
    }
    return 1U;
}
