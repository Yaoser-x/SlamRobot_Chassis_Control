#ifndef RELATIVE_YAW_CONTROL_H
#define RELATIVE_YAW_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  RELATIVE_YAW_END_NONE = 0,
  RELATIVE_YAW_END_COMPLETED,
  RELATIVE_YAW_END_TIMEOUT,
  RELATIVE_YAW_END_IMU_INVALID,
  RELATIVE_YAW_END_CONTROLLER_OFFLINE,
  RELATIVE_YAW_END_MANUAL_OVERRIDE,
  RELATIVE_YAW_END_SAFETY_STOP
} relative_yaw_end_reason_t;

typedef struct
{
  uint8_t active;
  uint8_t settling;
  relative_yaw_end_reason_t end_reason;
  float target_delta_deg;
  float accumulated_delta_deg;
  uint32_t start_ms;
  uint32_t timeout_ms;
  uint32_t settle_start_ms;
  uint32_t last_update_ms;
} relative_yaw_control_t;

void RelativeYawControl_Init(relative_yaw_control_t *control);
uint8_t RelativeYawControl_Start(relative_yaw_control_t *control,
                                 float target_delta_deg,
                                 float initial_yaw_deg,
                                 uint32_t now_ms,
                                 uint32_t timeout_ms);
uint8_t RelativeYawControl_Update(relative_yaw_control_t *control,
                                  float yaw_deg,
                                  float yaw_rate_dps,
                                  uint32_t now_ms,
                                  float *angular_z);
void RelativeYawControl_Cancel(relative_yaw_control_t *control,
                               relative_yaw_end_reason_t reason);

#ifdef __cplusplus
}
#endif

#endif
