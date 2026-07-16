#ifndef TELEOPERATION_STATUS_H
#define TELEOPERATION_STATUS_H

#include <stdint.h>

#define TELEOPERATION_HEADING_GATE_IMU_OFFLINE      (1UL << 0)
#define TELEOPERATION_HEADING_GATE_IMU_UNCALIBRATED (1UL << 1)
#define TELEOPERATION_HEADING_GATE_IMU_STALE        (1UL << 2)
#define TELEOPERATION_HEADING_GATE_IMU_QUALITY      (1UL << 3)

typedef struct
{
    uint8_t  online;
    uint8_t  analog_mode;
    uint8_t  cmd_dat_swapped;
    uint8_t  drive_enabled;
    uint8_t  btn1;
    uint8_t  btn2;
    uint8_t  left_x;
    uint8_t  left_y;
    uint8_t  right_x;
    uint8_t  right_y;
    uint8_t  macro_active;
    uint8_t  macro_button;
    uint8_t  heading_active;
    uint8_t  heading_end_reason;
    uint8_t  pressed_btn2;
    uint32_t heading_gate_flags;
    uint32_t imu_age_ms;
    float    heading_target_deg;
    float    heading_accumulated_deg;
    float    linear_x;
    float    angular_z;
    uint32_t rx_ok_count;
    uint32_t rx_fail_count;
    uint8_t  line_tracking_enabled;
    uint32_t generation;
} teleoperation_status_t;

#endif /* TELEOPERATION_STATUS_H */
