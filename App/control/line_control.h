#ifndef LINE_CONTROL_H
#define LINE_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float    line_position;
  float    error;
  uint8_t  detected_count;
  uint8_t  sensor_state[8];
  float    linear_x;
  float    angular_z;
  uint8_t  tracking_active;
  uint8_t  globally_enabled;
} line_control_state_t;

void     LineControl_Init(void);
void     LineControl_Update(void);
void     LineControl_Enable(uint8_t enable);
uint8_t  LineControl_IsEnabled(void);
void     LineControl_GetState(line_control_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
