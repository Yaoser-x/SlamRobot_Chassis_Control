#include "teleoperation_config.h"

uint8_t Teleoperation_ValidateConfig(const teleoperation_config_t *config)
{
    if (config == 0 || config->linear_max_mps <= 0.0f || config->angular_max_rps <= 0.0f || config->axis_deadzone < 0
        || config->axis_deadzone >= 127 || config->offline_fail_limit == 0U || config->heading_imu_fresh_ms == 0UL
        || config->idle_release_ms == 0UL)
    {
        return 0U;
    }
    return 1U;
}
