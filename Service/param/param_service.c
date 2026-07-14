#include "param_service.h"
#include "platform_critical.h"

#include "bsp_config.h"

#include "control_config.h"

#include <stddef.h>
#include <string.h>

static param_model_t current_params;
static uint8_t       current_params_initialized;
static uint32_t      current_params_generation;

typedef struct
{
    const char *name;
    float       min_value;
    float       max_value;
    float      *field;
} param_float_binding_t;

static uint8_t ParamService_Finite(float value)
{
    return (value > -3.4e38f && value < 3.4e38f) ? 1U : 0U;
}

void ParamService_Defaults(param_model_t *params)
{
    if (params == 0)
    {
        return;
    }

    *params                          = (param_model_t){0};
    params->version                  = PARAM_SERVICE_VERSION;
    params->max_linear_mps           = CHASSIS_MAX_LINEAR_MPS;
    params->max_angular_rps          = CHASSIS_MAX_ANGULAR_RPS;
    params->speed_ramp_mps2          = CHASSIS_SPEED_RAMP_MPS2;
    params->angular_ramp_rps2        = CHASSIS_ANGULAR_RAMP_RPS2;
    params->wheel_radius_m           = CHASSIS_WHEEL_RADIUS_M;
    params->track_width_m            = CHASSIS_TRACK_WIDTH_M;
    params->pid_kp[MOTOR_ID_M1]      = CHASSIS_PID_KP_M1;
    params->pid_ki[MOTOR_ID_M1]      = CHASSIS_PID_KI_M1;
    params->pid_kd[MOTOR_ID_M1]      = CHASSIS_PID_KD_M1;
    params->pid_kp[MOTOR_ID_M2]      = CHASSIS_PID_KP_M2;
    params->pid_ki[MOTOR_ID_M2]      = CHASSIS_PID_KI_M2;
    params->pid_kd[MOTOR_ID_M2]      = CHASSIS_PID_KD_M2;
    params->pid_kp[MOTOR_ID_M3]      = CHASSIS_PID_KP_M3;
    params->pid_ki[MOTOR_ID_M3]      = CHASSIS_PID_KI_M3;
    params->pid_kd[MOTOR_ID_M3]      = CHASSIS_PID_KD_M3;
    params->pid_kp[MOTOR_ID_M4]      = CHASSIS_PID_KP_M4;
    params->pid_ki[MOTOR_ID_M4]      = CHASSIS_PID_KI_M4;
    params->pid_kd[MOTOR_ID_M4]      = CHASSIS_PID_KD_M4;
    params->pid_integral_limit       = CHASSIS_PID_INTEGRAL_LIMIT;
    params->motor_dir[MOTOR_ID_M1]   = CHASSIS_M1_MOTOR_DIR;
    params->motor_dir[MOTOR_ID_M2]   = CHASSIS_M2_MOTOR_DIR;
    params->motor_dir[MOTOR_ID_M3]   = CHASSIS_M3_MOTOR_DIR;
    params->motor_dir[MOTOR_ID_M4]   = CHASSIS_M4_MOTOR_DIR;
    params->encoder_dir[MOTOR_ID_M1] = CHASSIS_M1_ENCODER_DIR;
    params->encoder_dir[MOTOR_ID_M2] = CHASSIS_M2_ENCODER_DIR;
    params->encoder_dir[MOTOR_ID_M3] = CHASSIS_M3_ENCODER_DIR;
    params->encoder_dir[MOTOR_ID_M4] = CHASSIS_M4_ENCODER_DIR;
    for (uint8_t i = 0U; i < PARAM_SERVICE_LINE_CHANNELS; ++i)
    {
        params->line_threshold_raw[i] = 500U;
    }
    params->line_active_low             = 1U;
    params->line_kp                     = 0.6f;
    params->line_kd                     = 0.05f;
    params->line_speed_mps              = 0.15f;
    params->line_slowdown_gain          = 0.7f;
    params->line_detect_debounce_frames = 2U;
    params->line_lost_debounce_frames   = 2U;
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        params->current_observe_a[i]    = 1.5f;
        params->current_soft_limit_a[i] = 2.0f;
        params->current_fault_a[i]      = 2.5f;
    }
    params->current_fault_debounce_ms             = 100U;
    params->straight_wheel_coupling_gain          = 0.30f;
    params->straight_heading_kp                   = 0.0f;
    params->straight_heading_ki                   = 0.0f;
    params->straight_heading_integral_limit_deg_s = 0.0f;
    params->straight_max_speed_mps                = 0.30f;
    params->straight_heading_hold_enabled         = 0U;
}

void ParamService_SetDefaults(void)
{
    param_model_t defaults;
    uint32_t      primask;

    ParamService_Defaults(&defaults);
    primask                    = PlatformCritical_Enter();
    current_params             = defaults;
    current_params_initialized = 1U;
    current_params_generation++;
    PlatformCritical_Exit(primask);
}

void ParamService_Get(param_model_t *params)
{
    (void)ParamService_GetSnapshot(params);
}

uint32_t ParamService_GetSnapshot(param_model_t *params)
{
    param_model_t defaults;
    uint32_t      generation;
    uint32_t      primask;

    if (params == 0)
    {
        return 0UL;
    }
    if (current_params_initialized == 0U)
    {
        ParamService_Defaults(&defaults);
        primask = PlatformCritical_Enter();
        if (current_params_initialized == 0U)
        {
            current_params             = defaults;
            current_params_initialized = 1U;
            current_params_generation++;
        }
        PlatformCritical_Exit(primask);
    }

    primask    = PlatformCritical_Enter();
    *params    = current_params;
    generation = current_params_generation;
    PlatformCritical_Exit(primask);
    return generation;
}

uint8_t ParamService_Validate(const param_model_t *params)
{
    if (params == 0 || params->version != PARAM_SERVICE_VERSION)
    {
        return 0U;
    }
    if (params->max_linear_mps <= 0.0f || params->max_linear_mps > 3.0f)
    {
        return 0U;
    }
    if (params->max_angular_rps <= 0.0f || params->max_angular_rps > 30.0f)
    {
        return 0U;
    }
    if (params->speed_ramp_mps2 <= 0.0f || params->speed_ramp_mps2 > 20.0f)
    {
        return 0U;
    }
    if (params->angular_ramp_rps2 <= 0.0f || params->angular_ramp_rps2 > 60.0f)
    {
        return 0U;
    }
    if (params->wheel_radius_m < 0.01f || params->wheel_radius_m > 0.20f)
    {
        return 0U;
    }
    if (params->track_width_m < 0.05f || params->track_width_m > 1.00f)
    {
        return 0U;
    }
    if (params->pid_integral_limit < 0.0f || params->pid_integral_limit > 5000.0f)
    {
        return 0U;
    }
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if ((params->motor_dir[i] != -1) && (params->motor_dir[i] != 1))
        {
            return 0U;
        }
        if ((params->encoder_dir[i] != -1) && (params->encoder_dir[i] != 1))
        {
            return 0U;
        }
        if (ParamService_Finite(params->pid_kp[i]) == 0U || ParamService_Finite(params->pid_ki[i]) == 0U
            || ParamService_Finite(params->pid_kd[i]) == 0U)
        {
            return 0U;
        }
    }
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        if (ParamService_Finite(params->imu_gyro_bias_dps[i]) == 0U)
        {
            return 0U;
        }
    }
    if (params->line_active_low > 1U || params->line_kp < 0.0f || params->line_kp > 10.0f || params->line_kd < 0.0f
        || params->line_kd > 10.0f || params->line_speed_mps < 0.01f || params->line_speed_mps > params->max_linear_mps
        || params->line_slowdown_gain < 0.0f || params->line_slowdown_gain > 5.0f
        || params->line_detect_debounce_frames == 0U || params->line_detect_debounce_frames > 20U
        || params->line_lost_debounce_frames == 0U || params->line_lost_debounce_frames > 20U
        || params->current_fault_debounce_ms < 20U || params->current_fault_debounce_ms > 2000U
        || params->straight_wheel_coupling_gain < 0.0f || params->straight_wheel_coupling_gain > 2.0f
        || params->straight_heading_kp < 0.0f || params->straight_heading_kp > 0.1f
        || params->straight_trim_forward_015_mps < -0.10f || params->straight_trim_forward_015_mps > 0.10f
        || params->straight_trim_forward_030_mps < -0.10f || params->straight_trim_forward_030_mps > 0.10f
        || params->straight_trim_reverse_015_mps < -0.10f || params->straight_trim_reverse_015_mps > 0.10f
        || params->straight_trim_reverse_030_mps < -0.10f || params->straight_trim_reverse_030_mps > 0.10f
        || params->straight_heading_ki < 0.0f || params->straight_heading_ki > 0.02f
        || params->straight_heading_integral_limit_deg_s < 0.0f || params->straight_heading_integral_limit_deg_s > 30.0f
        || params->straight_max_speed_mps < 0.05f || params->straight_max_speed_mps > 0.30f
        || params->straight_heading_hold_enabled > 1U)
    {
        return 0U;
    }
    for (uint8_t i = 0U; i < PARAM_SERVICE_LINE_CHANNELS; ++i)
    {
        if (params->line_threshold_raw[i] > 4095U)
        {
            return 0U;
        }
    }
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (params->current_observe_a[i] < 0.1f || params->current_soft_limit_a[i] < params->current_observe_a[i]
            || params->current_fault_a[i] < params->current_soft_limit_a[i] || params->current_fault_a[i] > 20.0f)
        {
            return 0U;
        }
    }
    return 1U;
}

uint8_t ParamService_Set(const param_model_t *params)
{
    uint32_t primask;

    if (ParamService_Validate(params) == 0U)
    {
        return 0U;
    }

    primask                    = PlatformCritical_Enter();
    current_params             = *params;
    current_params_initialized = 1U;
    current_params_generation++;
    PlatformCritical_Exit(primask);
    return 1U;
}

static uint8_t ParamService_FindFloat(param_model_t *params, const char *name, param_float_binding_t *binding)
{
    param_float_binding_t bindings[] = {
        {"max_linear_mps", 0.01f, 3.0f, &params->max_linear_mps},
        {"max_angular_rps", 0.01f, 30.0f, &params->max_angular_rps},
        {"speed_ramp_mps2", 0.01f, 20.0f, &params->speed_ramp_mps2},
        {"angular_ramp_rps2", 0.01f, 60.0f, &params->angular_ramp_rps2},
        {"wheel_radius_m", 0.01f, 0.20f, &params->wheel_radius_m},
        {"track_width_m", 0.05f, 1.00f, &params->track_width_m},
        {"pid_integral_limit", 0.0f, 5000.0f, &params->pid_integral_limit},
        {"line_kp", 0.0f, 10.0f, &params->line_kp},
        {"line_kd", 0.0f, 10.0f, &params->line_kd},
        {"line_speed_mps", 0.01f, 3.0f, &params->line_speed_mps},
        {"line_slowdown_gain", 0.0f, 5.0f, &params->line_slowdown_gain},
        {"straight_wheel_coupling_gain", 0.0f, 2.0f, &params->straight_wheel_coupling_gain},
        {"straight_heading_kp", 0.0f, 0.1f, &params->straight_heading_kp},
        {"straight_trim_forward_015_mps", -0.10f, 0.10f, &params->straight_trim_forward_015_mps},
        {"straight_trim_forward_030_mps", -0.10f, 0.10f, &params->straight_trim_forward_030_mps},
        {"straight_trim_reverse_015_mps", -0.10f, 0.10f, &params->straight_trim_reverse_015_mps},
        {"straight_trim_reverse_030_mps", -0.10f, 0.10f, &params->straight_trim_reverse_030_mps},
        {"straight_heading_ki", 0.0f, 0.02f, &params->straight_heading_ki},
        {"straight_heading_integral_limit_deg_s", 0.0f, 30.0f, &params->straight_heading_integral_limit_deg_s},
        {"straight_max_speed_mps", 0.05f, 0.30f, &params->straight_max_speed_mps},
    };

    for (uint8_t i = 0U; i < (uint8_t)(sizeof(bindings) / sizeof(bindings[0])); ++i)
    {
        if (strcmp(name, bindings[i].name) == 0)
        {
            *binding = bindings[i];
            return 1U;
        }
    }
    return 0U;
}

uint8_t ParamService_GetFloat(const param_model_t *params, const char *name, float *value)
{
    param_model_t         local;
    param_float_binding_t binding;

    if (params == 0 || name == 0 || value == 0)
    {
        return 0U;
    }
    local = *params;
    if (ParamService_FindFloat(&local, name, &binding) == 0U)
    {
        return 0U;
    }
    *value = *binding.field;
    return 1U;
}

uint8_t ParamService_SetFloat(param_model_t *params, const char *name, float value)
{
    param_float_binding_t binding;

    if (params == 0 || name == 0 || ParamService_Finite(value) == 0U)
    {
        return 0U;
    }
    if (ParamService_FindFloat(params, name, &binding) == 0U)
    {
        return 0U;
    }
    if (value < binding.min_value || value > binding.max_value)
    {
        return 0U;
    }
    *binding.field = value;
    return ParamService_Validate(params);
}

/* ---- int parameter bindings ---- */

typedef enum
{
    PARAM_INT_KIND_U8,
    PARAM_INT_KIND_U16
} param_int_kind_t;

typedef struct
{
    const char      *name;
    int32_t          min_value;
    int32_t          max_value;
    void            *field;
    param_int_kind_t kind;
} param_int_binding_t;

static uint8_t ParamService_FindInt(param_model_t *params, const char *name, param_int_binding_t *binding)
{
    param_int_binding_t bindings[] = {
        {"line_active_low", 0, 1, &params->line_active_low, PARAM_INT_KIND_U8},
        {"line_detect_debounce_frames", 1, 20, &params->line_detect_debounce_frames, PARAM_INT_KIND_U8},
        {"line_lost_debounce_frames", 1, 20, &params->line_lost_debounce_frames, PARAM_INT_KIND_U8},
        {"current_fault_debounce_ms", 20, 2000, &params->current_fault_debounce_ms, PARAM_INT_KIND_U16},
        {"straight_heading_hold_enabled", 0, 1, &params->straight_heading_hold_enabled, PARAM_INT_KIND_U8},
    };

    for (uint8_t i = 0U; i < (uint8_t)(sizeof(bindings) / sizeof(bindings[0])); ++i)
    {
        if (strcmp(name, bindings[i].name) == 0)
        {
            *binding = bindings[i];
            return 1U;
        }
    }
    return 0U;
}

uint8_t ParamService_GetInt(const param_model_t *params, const char *name, int32_t *value)
{
    param_model_t       local;
    param_int_binding_t binding;

    if (params == 0 || name == 0 || value == 0)
    {
        return 0U;
    }
    local = *params;
    if (ParamService_FindInt(&local, name, &binding) == 0U)
    {
        return 0U;
    }
    switch (binding.kind)
    {
        case PARAM_INT_KIND_U8:
            *value = (int32_t) * (uint8_t *)binding.field;
            return 1U;
        case PARAM_INT_KIND_U16:
            *value = (int32_t) * (uint16_t *)binding.field;
            return 1U;
        default:
            return 0U;
    }
}

uint8_t ParamService_SetInt(param_model_t *params, const char *name, int32_t value)
{
    param_int_binding_t binding;

    if (params == 0 || name == 0)
    {
        return 0U;
    }
    if (ParamService_FindInt(params, name, &binding) == 0U)
    {
        return 0U;
    }
    if (value < binding.min_value || value > binding.max_value)
    {
        return 0U;
    }
    switch (binding.kind)
    {
        case PARAM_INT_KIND_U8:
            *(uint8_t *)binding.field = (uint8_t)value;
            break;
        case PARAM_INT_KIND_U16:
            *(uint16_t *)binding.field = (uint16_t)value;
            break;
        default:
            return 0U;
    }
    return ParamService_Validate(params);
}
