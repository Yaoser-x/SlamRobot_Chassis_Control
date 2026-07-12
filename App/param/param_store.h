#ifndef PARAM_STORE_H
#define PARAM_STORE_H

#include <stdint.h>

#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PARAM_STORE_VERSION 3UL
#define PARAM_STORE_LINE_CHANNELS 8U

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
  uint16_t line_threshold_raw[PARAM_STORE_LINE_CHANNELS];
  uint8_t line_active_low;
  float line_kp;
  float line_kd;
  float line_speed_mps;
  float line_slowdown_gain;
  uint8_t line_detect_debounce_frames;
  uint8_t line_lost_debounce_frames;
  float current_observe_a[MOTOR_ID_COUNT];
  float current_soft_limit_a[MOTOR_ID_COUNT];
  float current_fault_a[MOTOR_ID_COUNT];
  uint16_t current_fault_debounce_ms;
  float straight_wheel_coupling_gain;
  float straight_heading_kp;
  float straight_trim_forward_015_mps;
  float straight_trim_forward_030_mps;
  float straight_trim_reverse_015_mps;
  float straight_trim_reverse_030_mps;
  float straight_heading_ki;
  float straight_heading_integral_limit_deg_s;
  float straight_max_speed_mps;
  uint8_t straight_heading_hold_enabled;
} param_store_t;

void ParamStore_Defaults(param_store_t *params);
void ParamStore_SetDefaults(void);
void ParamStore_Get(param_store_t *params);
uint32_t ParamStore_GetSnapshot(param_store_t *params);
uint8_t ParamStore_Set(const param_store_t *params);
uint8_t ParamStore_Validate(const param_store_t *params);
uint8_t ParamStore_GetFloat(const param_store_t *params, const char *name, float *value);
uint8_t ParamStore_SetFloat(param_store_t *params, const char *name, float value);
uint8_t ParamStore_GetInt(const param_store_t *params, const char *name, int32_t *value);
uint8_t ParamStore_SetInt(param_store_t *params, const char *name, int32_t value);

#ifdef __cplusplus
}
#endif

#endif /* PARAM_STORE_H */
