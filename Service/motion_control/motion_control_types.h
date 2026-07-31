#ifndef MOTION_CONTROL_TYPES_H
#define MOTION_CONTROL_TYPES_H

#include <stdint.h>

#include "command_management_types.h"
#include "parameter_management_types.h"
#include "power_management_status.h"
#include "safety_management_types.h"
#include "state_estimation_status.h"

typedef struct
{
    param_model_t value;
    uint32_t      sample_time_ms;
    uint32_t      generation;
    uint32_t      validity;
    uint32_t      quality;
} motion_parameter_fact_t;

typedef struct
{
    state_estimation_wheel_status_t value;
    uint32_t                        sample_time_ms;
    uint32_t                        generation;
    uint32_t                        validity;
    uint32_t                        quality;
} motion_wheel_fact_t;

typedef struct
{
    state_estimation_imu_status_t value;
    uint32_t                      sample_time_ms;
    uint32_t                      generation;
    uint32_t                      validity;
    uint32_t                      quality;
} motion_imu_fact_t;

typedef struct
{
    power_management_status_t value;
    uint32_t                  sample_time_ms;
    uint32_t                  generation;
    uint32_t                  validity;
    uint32_t                  quality;
} motion_power_fact_t;

typedef struct
{
    command_velocity_t value;
    uint32_t           sample_time_ms;
    uint32_t           generation;
    uint32_t           validity;
    uint32_t           quality;
    uint32_t           revoke_generation;
} motion_command_fact_t;

typedef struct
{
    safety_motion_permit_t value;
    uint32_t               sample_time_ms;
    uint32_t               generation;
    uint32_t               validity;
    uint32_t               quality;
} motion_safety_permit_fact_t;

typedef struct
{
    uint32_t                    now_ms;
    uint32_t                    nominal_period_ms;
    motion_parameter_fact_t     parameters;
    motion_wheel_fact_t         wheel;
    motion_imu_fact_t           imu;
    motion_power_fact_t         power;
    motion_command_fact_t       command;
    motion_safety_permit_fact_t safety_permit;
    uint8_t                     normal_motion_allowed;
    uint8_t                     diagnostic_motion_allowed;
} motion_control_input_t;

#define MOTION_EVENT_DRIVER_FAULT          (1UL << 0)
#define MOTION_EVENT_ENCODER_FEEDBACK_LOST (1UL << 1)
#define MOTION_EVENT_COMMAND_REVOKE        (1UL << 2)
#define MOTION_EVENT_SAFETY_PERMIT_STALE   (1UL << 3)
#define MOTION_EVENT_TEST_LEASE_EXPIRED    (1UL << 4)
#define MOTION_EVENT_SAFETY_PERMIT_CHANGED (1UL << 5)

typedef struct
{
    uint32_t flags;
    uint32_t occurred_at_ms;
    uint32_t generation;
} motion_control_event_t;

#endif /* MOTION_CONTROL_TYPES_H */
