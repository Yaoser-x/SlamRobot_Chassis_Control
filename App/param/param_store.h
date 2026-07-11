#ifndef PARAM_STORE_H
#define PARAM_STORE_H

#include <stdint.h>

#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PARAM_STORE_VERSION 1UL

typedef struct
{
  uint32_t version;
  float max_linear_mps;
  float max_angular_rps;
  float speed_ramp_mps2;
  float angular_ramp_rps2;
  float wheel_radius_m;
  float track_width_m;
  float pid_kp[MOTOR_ID_COUNT];
  float pid_ki[MOTOR_ID_COUNT];
  float pid_kd[MOTOR_ID_COUNT];
  float pid_integral_limit;
  int8_t motor_dir[MOTOR_ID_COUNT];
  int8_t encoder_dir[MOTOR_ID_COUNT];
  uint16_t current_zero_raw[MOTOR_ID_COUNT];
  uint8_t current_zero_valid;
  float imu_gyro_bias_dps[3];
  uint8_t imu_gyro_bias_valid;
} param_store_t;

void ParamStore_Defaults(param_store_t *params);
void ParamStore_SetDefaults(void);
void ParamStore_Get(param_store_t *params);
uint32_t ParamStore_GetSnapshot(param_store_t *params);
uint8_t ParamStore_Set(const param_store_t *params);
uint8_t ParamStore_Validate(const param_store_t *params);
uint8_t ParamStore_GetFloat(const param_store_t *params, const char *name, float *value);
uint8_t ParamStore_SetFloat(param_store_t *params, const char *name, float value);

#ifdef __cplusplus
}
#endif

#endif /* PARAM_STORE_H */
