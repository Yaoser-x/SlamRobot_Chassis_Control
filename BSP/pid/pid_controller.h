#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral_limit;
  float output_limit;
} pid_params_t;

typedef struct
{
  pid_params_t params;
  float integral;
  float prev_error;
  uint8_t initialized;
} pid_state_t;

void PidController_Init(pid_state_t *pid, const pid_params_t *params);
void PidController_Reset(pid_state_t *pid);
float PidController_Step(pid_state_t *pid, float target, float actual, float dt_s);
void PidController_SetParams(pid_state_t *pid, const pid_params_t *params);

#ifdef __cplusplus
}
#endif

#endif
