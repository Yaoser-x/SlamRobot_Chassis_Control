#ifndef TELEOPERATION_CONFIG_H
#define TELEOPERATION_CONFIG_H

#include <stdint.h>

/** @brief Product configuration for PS2 mapping and heading macros. */
typedef struct
{
    float    linear_max_mps;
    float    angular_max_rps;
    float    dpad_linear_mps;
    float    dpad_angular_rps;
    float    manual_cancel_threshold;
    uint32_t heading_quarter_timeout_ms;
    uint32_t heading_full_timeout_ms;
    uint32_t heading_imu_fresh_ms;
    uint32_t idle_release_ms;
    int16_t  axis_center;
    int16_t  axis_deadzone;
    uint8_t  offline_fail_limit;
    uint8_t  macro_l1_mask;
    uint8_t  macro_r1_mask;
    uint8_t  macro_l2_mask;
    uint8_t  macro_r2_mask;
    uint8_t  line_toggle_mask;
    uint8_t  linecal_floor_mask;
    uint8_t  linecal_line_mask;
} teleoperation_config_t;

#endif
