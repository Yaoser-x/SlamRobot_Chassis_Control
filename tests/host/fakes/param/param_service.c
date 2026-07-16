#include "param_service.h"

#include "bsp_config.h"
#include "control_config.h"
#include "parameter_management_service.h"

static void ParamService_BuildBeta4Config(parameter_management_config_t *config)
{
    param_model_t *params = &config->factory_defaults;

    *config                              = (parameter_management_config_t){0};
    params->version                      = PARAM_SERVICE_VERSION;
    params->max_linear_mps               = CHASSIS_MAX_LINEAR_MPS;
    params->max_angular_rps              = CHASSIS_MAX_ANGULAR_RPS;
    params->speed_ramp_mps2              = CHASSIS_SPEED_RAMP_MPS2;
    params->angular_ramp_rps2            = CHASSIS_ANGULAR_RAMP_RPS2;
    params->wheel_radius_m               = CHASSIS_WHEEL_RADIUS_M;
    params->track_width_m                = CHASSIS_TRACK_WIDTH_M;
    params->pid_kp[MOTOR_ID_M1]          = CHASSIS_PID_KP_M1;
    params->pid_ki[MOTOR_ID_M1]          = CHASSIS_PID_KI_M1;
    params->pid_kd[MOTOR_ID_M1]          = CHASSIS_PID_KD_M1;
    params->pid_kp[MOTOR_ID_M2]          = CHASSIS_PID_KP_M2;
    params->pid_ki[MOTOR_ID_M2]          = CHASSIS_PID_KI_M2;
    params->pid_kd[MOTOR_ID_M2]          = CHASSIS_PID_KD_M2;
    params->pid_kp[MOTOR_ID_M3]          = CHASSIS_PID_KP_M3;
    params->pid_ki[MOTOR_ID_M3]          = CHASSIS_PID_KI_M3;
    params->pid_kd[MOTOR_ID_M3]          = CHASSIS_PID_KD_M3;
    params->pid_kp[MOTOR_ID_M4]          = CHASSIS_PID_KP_M4;
    params->pid_ki[MOTOR_ID_M4]          = CHASSIS_PID_KI_M4;
    params->pid_kd[MOTOR_ID_M4]          = CHASSIS_PID_KD_M4;
    params->pid_integral_limit           = CHASSIS_PID_INTEGRAL_LIMIT;
    params->motor_dir[MOTOR_ID_M1]       = CHASSIS_M1_MOTOR_DIR;
    params->motor_dir[MOTOR_ID_M2]       = CHASSIS_M2_MOTOR_DIR;
    params->motor_dir[MOTOR_ID_M3]       = CHASSIS_M3_MOTOR_DIR;
    params->motor_dir[MOTOR_ID_M4]       = CHASSIS_M4_MOTOR_DIR;
    params->encoder_dir[MOTOR_ID_M1]     = CHASSIS_M1_ENCODER_DIR;
    params->encoder_dir[MOTOR_ID_M2]     = CHASSIS_M2_ENCODER_DIR;
    params->encoder_dir[MOTOR_ID_M3]     = CHASSIS_M3_ENCODER_DIR;
    params->encoder_dir[MOTOR_ID_M4]     = CHASSIS_M4_ENCODER_DIR;
    params->line_active_low              = 1U;
    params->line_kp                      = 0.6f;
    params->line_kd                      = 0.05f;
    params->line_speed_mps               = 0.15f;
    params->line_slowdown_gain           = 0.7f;
    params->line_detect_debounce_frames  = 2U;
    params->line_lost_debounce_frames    = 2U;
    params->current_fault_debounce_ms    = 100U;
    params->straight_wheel_coupling_gain = 0.30f;
    params->straight_max_speed_mps       = 0.30f;
    for (uint8_t index = 0U; index < PARAM_SERVICE_LINE_CHANNELS; ++index)
    {
        params->line_threshold_raw[index] = 500U;
    }
    for (uint8_t index = 0U; index < PARAM_MODEL_MOTOR_COUNT; ++index)
    {
        params->current_observe_a[index]    = 1.5f;
        params->current_soft_limit_a[index] = 2.0f;
        params->current_fault_a[index]      = 2.5f;
    }
    config->load_flash_on_boot      = 1U;
    config->persist_imu_calibration = 1U;
    config->persist_current_zero    = 1U;
}

static void ParamService_EnsureInitialized(void)
{
    parameter_management_status_t status;

    (void)ParameterManagement_GetStatus(&status);
    if (status.initialized == 0U)
    {
        parameter_management_config_t config;
        ParamService_BuildBeta4Config(&config);
        (void)ParameterManagement_Init(&config);
    }
}

void ParamService_Defaults(param_model_t *params)
{
    ParamService_EnsureInitialized();
    ParameterManagement_Defaults(params);
}

void ParamService_SetDefaults(void)
{
    ParamService_EnsureInitialized();
    ParameterManagement_ResetToDefaults();
}

void ParamService_Get(param_model_t *params)
{
    (void)ParamService_GetSnapshot(params);
}

uint32_t ParamService_GetSnapshot(param_model_t *params)
{
    ParamService_EnsureInitialized();
    return ParameterManagement_GetSnapshot(params);
}

uint8_t ParamService_Set(const param_model_t *params)
{
    ParamService_EnsureInitialized();
    return ParameterManagement_Set(params);
}

uint8_t ParamService_Validate(const param_model_t *params)
{
    return ParameterManagement_Validate(params);
}

uint8_t ParamService_GetFloat(const param_model_t *params, const char *name, float *value)
{
    return ParameterManagement_GetFloat(params, name, value);
}

uint8_t ParamService_SetFloat(param_model_t *params, const char *name, float value)
{
    return ParameterManagement_SetFloat(params, name, value);
}

uint8_t ParamService_GetInt(const param_model_t *params, const char *name, int32_t *value)
{
    return ParameterManagement_GetInt(params, name, value);
}

uint8_t ParamService_SetInt(param_model_t *params, const char *name, int32_t value)
{
    return ParameterManagement_SetInt(params, name, value);
}
