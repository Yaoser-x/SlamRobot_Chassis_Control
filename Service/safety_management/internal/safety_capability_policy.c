#include "safety_capability_policy.h"

#include "state_estimation_status.h"

#define SAFETY_HEADING_BLOCKING_QUALITY                                                                                \
    (STATE_ESTIMATION_IMU_QUALITY_SPI_ERROR | STATE_ESTIMATION_IMU_QUALITY_INIT_FAILED                                 \
     | STATE_ESTIMATION_IMU_QUALITY_TIMESTAMP_ERROR | STATE_ESTIMATION_IMU_QUALITY_PROFILE_MISMATCH                    \
     | STATE_ESTIMATION_IMU_QUALITY_GYRO_SATURATION)

void SafetyCapabilityPolicy_Evaluate(const safety_capability_input_t  *input,
                                     const safety_management_config_t *config,
                                     safety_capability_permit_t       *permit)
{
    uint8_t base;
    uint8_t heading;
    uint8_t hardware;

    if (permit == 0)
    {
        return;
    }
    *permit = (safety_capability_permit_t){0};
    if (input == 0 || config == 0)
    {
        return;
    }
    hardware = (input->parameter_ready != 0U && input->current_control_valid != 0U && input->motor_driver_ok != 0U
                && input->tim_break_clear != 0U && input->safety_fault_free != 0U && input->fatal_post_flags == 0UL)
                   ? 1U
                   : 0U;
    base     = (hardware != 0U && input->encoder_valid != 0U) ? 1U : 0U;
    heading  = (input->imu_online != 0U && input->imu_fresh != 0U && input->imu_calibrated != 0U
               && (input->imu_quality_flags & SAFETY_HEADING_BLOCKING_QUALITY) == 0UL)
                   ? 1U
                   : 0U;

    permit->base_motion     = base;
    permit->manual          = base;
    permit->remote_velocity = (base != 0U && (config->remote_velocity_requires_imu == 0U || heading != 0U)) ? 1U : 0U;
    permit->line            = (base != 0U && input->line_sensor_valid != 0U) ? 1U : 0U;
    permit->heading_assist  = heading;
    permit->heading_macro   = heading;
    permit->maintenance_motion = hardware;
}
