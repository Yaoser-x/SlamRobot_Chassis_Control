#include "param_store.h"

#include "bsp_config.h"
#include "chassis_config.h"

#include <string.h>

static param_store_t current_params;
static uint8_t current_params_initialized;

typedef struct
{
  const char *name;
  float min_value;
  float max_value;
  float *field;
} param_float_binding_t;

static uint8_t ParamStore_Finite(float value)
{
  return (value > -3.4e38f && value < 3.4e38f) ? 1U : 0U;
}

void ParamStore_Defaults(param_store_t *params)
{
  if (params == 0)
  {
    return;
  }

  *params = (param_store_t){0};
  params->version = PARAM_STORE_VERSION;
  params->max_linear_mps = CHASSIS_MAX_LINEAR_MPS;
  params->max_angular_rps = CHASSIS_MAX_ANGULAR_RPS;
  params->speed_ramp_mps2 = CHASSIS_SPEED_RAMP_MPS2;
  params->angular_ramp_rps2 = CHASSIS_ANGULAR_RAMP_RPS2;
  params->wheel_radius_m = CHASSIS_WHEEL_RADIUS_M;
  params->track_width_m = CHASSIS_TRACK_WIDTH_M;
  params->pid_kp[MOTOR_ID_M1] = CHASSIS_PID_KP_M1;
  params->pid_ki[MOTOR_ID_M1] = CHASSIS_PID_KI_M1;
  params->pid_kd[MOTOR_ID_M1] = CHASSIS_PID_KD_M1;
  params->pid_kp[MOTOR_ID_M2] = CHASSIS_PID_KP_M2;
  params->pid_ki[MOTOR_ID_M2] = CHASSIS_PID_KI_M2;
  params->pid_kd[MOTOR_ID_M2] = CHASSIS_PID_KD_M2;
  params->pid_kp[MOTOR_ID_M3] = CHASSIS_PID_KP_M3;
  params->pid_ki[MOTOR_ID_M3] = CHASSIS_PID_KI_M3;
  params->pid_kd[MOTOR_ID_M3] = CHASSIS_PID_KD_M3;
  params->pid_kp[MOTOR_ID_M4] = CHASSIS_PID_KP_M4;
  params->pid_ki[MOTOR_ID_M4] = CHASSIS_PID_KI_M4;
  params->pid_kd[MOTOR_ID_M4] = CHASSIS_PID_KD_M4;
  params->pid_integral_limit = CHASSIS_PID_INTEGRAL_LIMIT;
  params->motor_dir[MOTOR_ID_M1] = CHASSIS_M1_MOTOR_DIR;
  params->motor_dir[MOTOR_ID_M2] = CHASSIS_M2_MOTOR_DIR;
  params->motor_dir[MOTOR_ID_M3] = CHASSIS_M3_MOTOR_DIR;
  params->motor_dir[MOTOR_ID_M4] = CHASSIS_M4_MOTOR_DIR;
  params->encoder_dir[MOTOR_ID_M1] = CHASSIS_M1_ENCODER_DIR;
  params->encoder_dir[MOTOR_ID_M2] = CHASSIS_M2_ENCODER_DIR;
  params->encoder_dir[MOTOR_ID_M3] = CHASSIS_M3_ENCODER_DIR;
  params->encoder_dir[MOTOR_ID_M4] = CHASSIS_M4_ENCODER_DIR;
}

void ParamStore_SetDefaults(void)
{
  ParamStore_Defaults(&current_params);
  current_params_initialized = 1U;
}

void ParamStore_Get(param_store_t *params)
{
  if (params == 0)
  {
    return;
  }
  if (current_params_initialized == 0U)
  {
    ParamStore_SetDefaults();
  }
  *params = current_params;
}

uint8_t ParamStore_Validate(const param_store_t *params)
{
  if (params == 0 || params->version != PARAM_STORE_VERSION)
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
    if (ParamStore_Finite(params->pid_kp[i]) == 0U ||
        ParamStore_Finite(params->pid_ki[i]) == 0U ||
        ParamStore_Finite(params->pid_kd[i]) == 0U)
    {
      return 0U;
    }
  }
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    if (ParamStore_Finite(params->imu_gyro_bias_dps[i]) == 0U)
    {
      return 0U;
    }
  }
  return 1U;
}

uint8_t ParamStore_Set(const param_store_t *params)
{
  if (ParamStore_Validate(params) == 0U)
  {
    return 0U;
  }
  current_params = *params;
  current_params_initialized = 1U;
  return 1U;
}

static uint8_t ParamStore_FindFloat(param_store_t *params,
                                    const char *name,
                                    param_float_binding_t *binding)
{
  param_float_binding_t bindings[] = {
    { "max_linear_mps", 0.01f, 3.0f, &params->max_linear_mps },
    { "max_angular_rps", 0.01f, 30.0f, &params->max_angular_rps },
    { "speed_ramp_mps2", 0.01f, 20.0f, &params->speed_ramp_mps2 },
    { "angular_ramp_rps2", 0.01f, 60.0f, &params->angular_ramp_rps2 },
    { "wheel_radius_m", 0.01f, 0.20f, &params->wheel_radius_m },
    { "track_width_m", 0.05f, 1.00f, &params->track_width_m },
    { "pid_integral_limit", 0.0f, 5000.0f, &params->pid_integral_limit },
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

uint8_t ParamStore_GetFloat(const param_store_t *params, const char *name, float *value)
{
  param_store_t local;
  param_float_binding_t binding;

  if (params == 0 || name == 0 || value == 0)
  {
    return 0U;
  }
  local = *params;
  if (ParamStore_FindFloat(&local, name, &binding) == 0U)
  {
    return 0U;
  }
  *value = *binding.field;
  return 1U;
}

uint8_t ParamStore_SetFloat(param_store_t *params, const char *name, float value)
{
  param_float_binding_t binding;

  if (params == 0 || name == 0 || ParamStore_Finite(value) == 0U)
  {
    return 0U;
  }
  if (ParamStore_FindFloat(params, name, &binding) == 0U)
  {
    return 0U;
  }
  if (value < binding.min_value || value > binding.max_value)
  {
    return 0U;
  }
  *binding.field = value;
  return ParamStore_Validate(params);
}
