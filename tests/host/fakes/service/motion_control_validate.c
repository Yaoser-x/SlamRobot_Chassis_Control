#include "motion_control_config.h"

uint8_t MotionControl_ValidateConfig(const motion_control_config_t *config)
{
    if (config == 0 || config->max_linear_mps <= 0.0f || config->max_angular_rps <= 0.0f
        || config->open_loop_full_mps <= 0.0f || config->pid_correction_limit <= 0.0f
        || config->test_mode_lease_ms == 0UL || config->encoder_feedback_timeout_ms == 0UL
        || config->pid_feedback_loss_count == 0U)
    {
        return 0U;
    }
    return 1U;
}
