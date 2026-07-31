#ifndef CONTROL_MODE_CONFIG_H
#define CONTROL_MODE_CONFIG_H

#include <stdint.h>

/** Non-persistent product policy for PS2 takeover and manual release. */
typedef struct
{
    float    takeover_enter_threshold;
    float    takeover_exit_threshold;
    uint32_t manual_neutral_restore_ms;
    uint8_t  takeover_confirm_samples;
} control_mode_config_t;

uint8_t ControlModeConfig_Validate(const control_mode_config_t *config);

#endif /* CONTROL_MODE_CONFIG_H */
