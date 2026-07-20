#include "parameter_identity_crc.h"

#include <string.h>

static uint32_t ParameterIdentityCrc_UpdateByte(uint32_t crc, uint8_t value)
{
    crc ^= value;
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
        crc = ((crc & 1UL) != 0UL) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
    }
    return crc;
}

static void ParameterIdentityCrc_U8(uint32_t *crc, uint8_t value)
{
    *crc = ParameterIdentityCrc_UpdateByte(*crc, value);
}

static void ParameterIdentityCrc_U16(uint32_t *crc, uint16_t value)
{
    ParameterIdentityCrc_U8(crc, (uint8_t)value);
    ParameterIdentityCrc_U8(crc, (uint8_t)(value >> 8));
}

static void ParameterIdentityCrc_U32(uint32_t *crc, uint32_t value)
{
    ParameterIdentityCrc_U16(crc, (uint16_t)value);
    ParameterIdentityCrc_U16(crc, (uint16_t)(value >> 16));
}

static void ParameterIdentityCrc_Float(uint32_t *crc, float value)
{
    uint32_t raw;

    (void)memcpy(&raw, &value, sizeof(raw));
    ParameterIdentityCrc_U32(crc, raw);
}

uint32_t ParameterIdentityCrc_Calculate(const param_model_t *params)
{
    uint32_t crc = 0xFFFFFFFFUL;

    if (params == 0)
    {
        return 0UL;
    }

    ParameterIdentityCrc_U32(&crc, params->version);
    ParameterIdentityCrc_Float(&crc, params->max_linear_mps);
    ParameterIdentityCrc_Float(&crc, params->max_angular_rps);
    ParameterIdentityCrc_Float(&crc, params->speed_ramp_mps2);
    ParameterIdentityCrc_Float(&crc, params->angular_ramp_rps2);
    ParameterIdentityCrc_Float(&crc, params->wheel_radius_m);
    ParameterIdentityCrc_Float(&crc, params->track_width_m);
    for (uint8_t i = 0U; i < PARAM_MODEL_MOTOR_COUNT; ++i)
    {
        ParameterIdentityCrc_Float(&crc, params->pid_kp[i]);
    }
    for (uint8_t i = 0U; i < PARAM_MODEL_MOTOR_COUNT; ++i)
    {
        ParameterIdentityCrc_Float(&crc, params->pid_ki[i]);
    }
    for (uint8_t i = 0U; i < PARAM_MODEL_MOTOR_COUNT; ++i)
    {
        ParameterIdentityCrc_Float(&crc, params->pid_kd[i]);
    }
    ParameterIdentityCrc_Float(&crc, params->pid_integral_limit);
    for (uint8_t i = 0U; i < PARAM_MODEL_MOTOR_COUNT; ++i)
    {
        ParameterIdentityCrc_U8(&crc, (uint8_t)params->motor_dir[i]);
    }
    for (uint8_t i = 0U; i < PARAM_MODEL_MOTOR_COUNT; ++i)
    {
        ParameterIdentityCrc_U8(&crc, (uint8_t)params->encoder_dir[i]);
    }
    for (uint8_t i = 0U; i < PARAM_MODEL_MOTOR_COUNT; ++i)
    {
        ParameterIdentityCrc_U16(&crc, params->current_zero_raw[i]);
    }
    ParameterIdentityCrc_U8(&crc, params->current_zero_valid);
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        ParameterIdentityCrc_Float(&crc, params->imu_gyro_bias_dps[i]);
    }
    ParameterIdentityCrc_U8(&crc, params->imu_gyro_bias_valid);
    for (uint8_t i = 0U; i < PARAM_MODEL_LINE_CHANNELS; ++i)
    {
        ParameterIdentityCrc_U16(&crc, params->line_threshold_raw[i]);
    }
    ParameterIdentityCrc_U8(&crc, params->line_active_low);
    ParameterIdentityCrc_Float(&crc, params->line_kp);
    ParameterIdentityCrc_Float(&crc, params->line_kd);
    ParameterIdentityCrc_Float(&crc, params->line_speed_mps);
    ParameterIdentityCrc_Float(&crc, params->line_slowdown_gain);
    ParameterIdentityCrc_U8(&crc, params->line_detect_debounce_frames);
    ParameterIdentityCrc_U8(&crc, params->line_lost_debounce_frames);
    for (uint8_t i = 0U; i < PARAM_MODEL_MOTOR_COUNT; ++i)
    {
        ParameterIdentityCrc_Float(&crc, params->current_observe_a[i]);
    }
    for (uint8_t i = 0U; i < PARAM_MODEL_MOTOR_COUNT; ++i)
    {
        ParameterIdentityCrc_Float(&crc, params->current_soft_limit_a[i]);
    }
    for (uint8_t i = 0U; i < PARAM_MODEL_MOTOR_COUNT; ++i)
    {
        ParameterIdentityCrc_Float(&crc, params->current_fault_a[i]);
    }
    ParameterIdentityCrc_U16(&crc, params->current_fault_debounce_ms);
    ParameterIdentityCrc_Float(&crc, params->straight_wheel_coupling_gain);
    ParameterIdentityCrc_Float(&crc, params->straight_heading_kp);
    ParameterIdentityCrc_Float(&crc, params->straight_trim_forward_015_mps);
    ParameterIdentityCrc_Float(&crc, params->straight_trim_forward_030_mps);
    ParameterIdentityCrc_Float(&crc, params->straight_trim_reverse_015_mps);
    ParameterIdentityCrc_Float(&crc, params->straight_trim_reverse_030_mps);
    ParameterIdentityCrc_Float(&crc, params->straight_heading_ki);
    ParameterIdentityCrc_Float(&crc, params->straight_heading_integral_limit_deg_s);
    ParameterIdentityCrc_Float(&crc, params->straight_max_speed_mps);
    ParameterIdentityCrc_U8(&crc, params->straight_heading_hold_enabled);
    return crc ^ 0xFFFFFFFFUL;
}
