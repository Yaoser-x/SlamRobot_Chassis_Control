#include "flash_param_migration.h"
#include "parameter_management_service.h"

#include <string.h>

void FlashParamMigrate_FromV1(const param_model_v1_t *old, param_model_t *params)
{
    ParameterManagement_Defaults(params);
    params->max_linear_mps     = old->max_linear_mps;
    params->max_angular_rps    = old->max_angular_rps;
    params->speed_ramp_mps2    = old->speed_ramp_mps2;
    params->angular_ramp_rps2  = old->angular_ramp_rps2;
    params->wheel_radius_m     = old->wheel_radius_m;
    params->track_width_m      = old->track_width_m;
    params->pid_integral_limit = old->pid_integral_limit;
    for (uint8_t i = 0U; i < PARAM_MODEL_MOTOR_COUNT; ++i)
    {
        params->pid_kp[i]           = old->pid_kp[i];
        params->pid_ki[i]           = old->pid_ki[i];
        params->pid_kd[i]           = old->pid_kd[i];
        params->motor_dir[i]        = old->motor_dir[i];
        params->encoder_dir[i]      = old->encoder_dir[i];
        params->current_zero_raw[i] = old->current_zero_raw[i];
    }
    params->current_zero_valid = old->current_zero_valid;
}

void FlashParamMigrate_FromV2(const flash_param_bundle_v2_t *old, flash_param_bundle_t *bundle)
{
    FlashParamMigrate_FromV1(&old->params, &bundle->params);
    bundle->imu_calibration = old->imu_calibration;
    if (ImuBmi270Calibration_Validate(&bundle->imu_calibration) == 0U)
    {
        ImuBmi270Calibration_Default(&bundle->imu_calibration);
    }
    if (old->params.imu_gyro_bias_valid != 0U && bundle->imu_calibration.gyro_bias_dps[0] == 0.0f
        && bundle->imu_calibration.gyro_bias_dps[1] == 0.0f && bundle->imu_calibration.gyro_bias_dps[2] == 0.0f)
    {
        for (uint8_t i = 0U; i < 3U; ++i)
        {
            bundle->imu_calibration.gyro_bias_dps[i] = old->params.imu_gyro_bias_dps[i];
        }
        bundle->imu_calibration.crc = ImuBmi270Calibration_Crc(&bundle->imu_calibration);
    }
    bundle->params.imu_gyro_bias_valid = 0U;
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        bundle->params.imu_gyro_bias_dps[i] = 0.0f;
    }
}

void FlashParamMigrate_FromV3(const flash_param_bundle_v3_t *old, flash_param_bundle_t *bundle)
{
    ParameterManagement_Defaults(&bundle->params);
    bundle->params.max_linear_mps    = old->params.max_linear_mps;
    bundle->params.max_angular_rps   = old->params.max_angular_rps;
    bundle->params.speed_ramp_mps2   = old->params.speed_ramp_mps2;
    bundle->params.angular_ramp_rps2 = old->params.angular_ramp_rps2;
    bundle->params.wheel_radius_m    = old->params.wheel_radius_m;
    bundle->params.track_width_m     = old->params.track_width_m;
    memcpy(bundle->params.pid_kp, old->params.pid_kp, sizeof(bundle->params.pid_kp));
    memcpy(bundle->params.pid_ki, old->params.pid_ki, sizeof(bundle->params.pid_ki));
    memcpy(bundle->params.pid_kd, old->params.pid_kd, sizeof(bundle->params.pid_kd));
    bundle->params.pid_integral_limit = old->params.pid_integral_limit;
    memcpy(bundle->params.motor_dir, old->params.motor_dir, sizeof(bundle->params.motor_dir));
    memcpy(bundle->params.encoder_dir, old->params.encoder_dir, sizeof(bundle->params.encoder_dir));
    memcpy(bundle->params.current_zero_raw, old->params.current_zero_raw, sizeof(bundle->params.current_zero_raw));
    bundle->params.current_zero_valid = old->params.current_zero_valid;
    memcpy(bundle->params.line_threshold_raw,
           old->params.line_threshold_raw,
           sizeof(bundle->params.line_threshold_raw));
    bundle->params.line_active_low             = old->params.line_active_low;
    bundle->params.line_kp                     = old->params.line_kp;
    bundle->params.line_kd                     = old->params.line_kd;
    bundle->params.line_speed_mps              = old->params.line_speed_mps;
    bundle->params.line_slowdown_gain          = old->params.line_slowdown_gain;
    bundle->params.line_detect_debounce_frames = old->params.line_detect_debounce_frames;
    bundle->params.line_lost_debounce_frames   = old->params.line_lost_debounce_frames;
    memcpy(bundle->params.current_observe_a, old->params.current_observe_a, sizeof(bundle->params.current_observe_a));
    memcpy(bundle->params.current_soft_limit_a,
           old->params.current_soft_limit_a,
           sizeof(bundle->params.current_soft_limit_a));
    memcpy(bundle->params.current_fault_a, old->params.current_fault_a, sizeof(bundle->params.current_fault_a));
    bundle->params.current_fault_debounce_ms     = old->params.current_fault_debounce_ms;
    bundle->params.straight_wheel_coupling_gain  = old->params.straight_wheel_coupling_gain;
    bundle->params.straight_heading_kp           = old->params.straight_heading_kp;
    bundle->params.straight_heading_hold_enabled = 0U;
    bundle->imu_calibration                      = old->imu_calibration;
    if (ImuBmi270Calibration_Validate(&bundle->imu_calibration) == 0U)
    {
        ImuBmi270Calibration_Default(&bundle->imu_calibration);
    }
}
