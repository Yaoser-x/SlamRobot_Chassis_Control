#ifndef SAFETY_MANAGEMENT_CONFIG_H
#define SAFETY_MANAGEMENT_CONFIG_H

#include <stdint.h>

#define SAFETY_MANAGEMENT_MOTOR_COUNT 4U

/** @brief Product configuration consumed by Safety Management. */
typedef struct
{
    float    battery_low_warn_v;
    float    battery_low_clear_v;
    float    battery_critical_v;
    float    battery_recover_v;
    uint32_t battery_critical_debounce_ms;
    uint32_t battery_recover_debounce_ms;
    uint32_t update_period_ms;
    uint32_t overcurrent_startup_blank_ms;
    uint32_t overcurrent_startup_rearm_ms;
    uint8_t  battery_low_monitor_enabled;
    uint8_t  overcurrent_fault_enabled;
    float    current_observe_a[SAFETY_MANAGEMENT_MOTOR_COUNT];
    float    current_fault_a[SAFETY_MANAGEMENT_MOTOR_COUNT];
    uint16_t current_fault_debounce_ms;
} safety_management_config_t;

#endif
