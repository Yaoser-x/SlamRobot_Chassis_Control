#include <stdio.h>
#include <stdlib.h>

#include "safety_capability_policy.h"

static void require_int(int condition, const char *message)
{
    if (condition == 0)
    {
        (void)printf("FAIL: %s\n", message);
        exit(1);
    }
}

static safety_capability_input_t valid_input(void)
{
    return (safety_capability_input_t){
        .parameter_ready       = 1U,
        .current_control_valid = 1U,
        .encoder_valid         = 1U,
        .motor_driver_ok       = 1U,
        .tim_break_clear       = 1U,
        .safety_fault_free     = 1U,
        .line_sensor_valid     = 1U,
        .imu_online            = 1U,
        .imu_fresh             = 1U,
        .imu_calibrated        = 1U,
    };
}

int main(void)
{
    safety_capability_input_t  input  = valid_input();
    safety_management_config_t config = {.remote_velocity_requires_imu = 0U};
    safety_capability_permit_t permit;

    SafetyCapabilityPolicy_Evaluate(&input, &config, &permit);
    require_int(permit.base_motion != 0U && permit.manual != 0U && permit.remote_velocity != 0U,
                "healthy platform permits base manual and remote motion");
    require_int(permit.line != 0U && permit.heading_assist != 0U && permit.heading_macro != 0U,
                "sensor capabilities are independently published");

    input.imu_online = 0U;
    SafetyCapabilityPolicy_Evaluate(&input, &config, &permit);
    require_int(permit.base_motion != 0U && permit.remote_velocity != 0U,
                "default product policy degrades IMU without blocking remote velocity");
    require_int(permit.heading_assist == 0U && permit.heading_macro == 0U, "IMU failure revokes heading only");

    config.remote_velocity_requires_imu = 1U;
    SafetyCapabilityPolicy_Evaluate(&input, &config, &permit);
    require_int(permit.remote_velocity == 0U && permit.manual != 0U,
                "compile-time product policy may require IMU for remote velocity");

    input               = valid_input();
    input.encoder_valid = 0U;
    SafetyCapabilityPolicy_Evaluate(&input, &config, &permit);
    require_int(permit.base_motion == 0U && permit.maintenance_motion != 0U,
                "restricted maintenance capability is independent of encoder feedback");

    input                   = valid_input();
    input.line_sensor_valid = 0U;
    SafetyCapabilityPolicy_Evaluate(&input, &config, &permit);
    require_int(permit.base_motion != 0U && permit.line == 0U, "line quality only revokes line capability");

    input                  = valid_input();
    input.fatal_post_flags = 1UL;
    SafetyCapabilityPolicy_Evaluate(&input, &config, &permit);
    require_int(permit.base_motion == 0U && permit.maintenance_motion == 0U, "fatal POST revokes hardware motion");
    return 0;
}
