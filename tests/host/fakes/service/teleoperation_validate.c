#include "teleoperation_config.h"

uint8_t Teleoperation_ValidateConfig(const teleoperation_config_t *config)
{
    if (config == 0 || !(config->linear_max_mps > 0.0f && config->linear_max_mps <= 10.0f)
        || !(config->angular_max_rps > 0.0f && config->angular_max_rps <= 50.0f)
        || !(config->manual_cancel_threshold >= 0.0f && config->manual_cancel_threshold <= 1.0f)
        || !(config->slow_speed_scale >= 0.0f && config->slow_speed_scale <= 1.0f)
        || (config->slow_button_mask != 0U && config->slow_speed_scale == 0.0f) || config->axis_deadzone < 0
        || config->axis_deadzone >= 127 || config->offline_fail_limit == 0U || config->heading_imu_fresh_ms == 0UL
        || config->idle_release_ms == 0UL)
    {
        return 0U;
    }
    return 1U;
}
