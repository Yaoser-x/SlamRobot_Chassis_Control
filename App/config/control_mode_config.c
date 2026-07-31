#include "control_mode_config.h"

uint8_t ControlModeConfig_Validate(const control_mode_config_t *config)
{
    if (config == 0 || !(config->takeover_enter_threshold > 0.0f) || !(config->takeover_enter_threshold <= 1.0f)
        || !(config->takeover_exit_threshold >= 0.0f)
        || !(config->takeover_exit_threshold < config->takeover_enter_threshold)
        || config->takeover_confirm_samples == 0U || config->manual_neutral_restore_ms == 0UL)
    {
        return 0U;
    }
    return 1U;
}
