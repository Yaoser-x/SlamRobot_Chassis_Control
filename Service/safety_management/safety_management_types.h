#ifndef SAFETY_MANAGEMENT_TYPES_H
#define SAFETY_MANAGEMENT_TYPES_H

#include <stdint.h>

#include "command_management_status.h"
#include "power_management_status.h"
#include "safety_management_config.h"
#include "state_estimation_status.h"
#include "system_monitoring_status.h"

typedef struct
{
    uint8_t base_motion;
    uint8_t manual;
    uint8_t remote_velocity;
    uint8_t line;
    uint8_t heading_assist;
    uint8_t heading_macro;
    uint8_t maintenance_motion;
} safety_capability_permit_t;

typedef struct
{
    uint8_t  base_motion;
    uint8_t  heading_assist;
    uint8_t  maintenance_motion;
    uint32_t generation;
    uint32_t issued_at_ms;
    uint32_t valid_for_ms;
} safety_motion_permit_t;

typedef struct
{
    uint8_t  parameter_ready;
    uint8_t  current_control_valid;
    uint8_t  encoder_valid;
    uint8_t  motor_driver_ok;
    uint8_t  tim_break_clear;
    uint8_t  safety_fault_free;
    uint8_t  line_sensor_valid;
    uint8_t  imu_online;
    uint8_t  imu_fresh;
    uint8_t  imu_calibrated;
    uint32_t imu_quality_flags;
    uint32_t fatal_post_flags;
} safety_capability_input_t;

typedef struct
{
    power_management_status_t value;
    uint32_t                  sample_time_ms;
    uint32_t                  generation;
    uint32_t                  validity;
    uint32_t                  quality;
} safety_power_fact_t;

typedef struct
{
    state_estimation_wheel_status_t value;
    uint32_t                        sample_time_ms;
    uint32_t                        generation;
    uint32_t                        validity;
    uint32_t                        quality;
} safety_wheel_fact_t;

typedef struct
{
    int16_t requested_pwm[SAFETY_MANAGEMENT_MOTOR_COUNT];
    int16_t applied_pwm[SAFETY_MANAGEMENT_MOTOR_COUNT];
    int16_t effective_pwm[SAFETY_MANAGEMENT_MOTOR_COUNT];
    uint8_t fault_active[SAFETY_MANAGEMENT_MOTOR_COUNT];
    uint8_t tim1_break_latched;
    uint8_t enabled_mask;
} safety_motor_value_t;

typedef struct
{
    safety_motor_value_t value;
    uint32_t             sample_time_ms;
    uint32_t             generation;
    uint32_t             validity;
    uint32_t             quality;
} safety_motor_fact_t;

typedef struct
{
    command_management_status_t value;
    uint32_t                    sample_time_ms;
    uint32_t                    generation;
    uint32_t                    validity;
    uint32_t                    quality;
} safety_command_fact_t;

typedef struct
{
    system_monitoring_status_t value;
    uint32_t                   sample_time_ms;
    uint32_t                   generation;
    uint32_t                   validity;
    uint32_t                   quality;
} safety_system_fact_t;

typedef struct
{
    uint32_t              now_ms;
    safety_power_fact_t   power;
    safety_wheel_fact_t   wheel;
    safety_motor_fact_t   motor;
    safety_command_fact_t command;
    safety_system_fact_t  system;
} safety_management_input_t;

typedef struct
{
    safety_wheel_fact_t wheel;
    safety_motor_fact_t motor;
    uint8_t             tim_break_clear_succeeded;
} safety_clear_input_t;

#endif /* SAFETY_MANAGEMENT_TYPES_H */
