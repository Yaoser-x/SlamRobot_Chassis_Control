#ifndef PS2_CONTROL_H
#define PS2_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t online;
  uint8_t analog_mode;
  uint8_t cmd_dat_swapped;
  uint8_t drive_enabled;
  uint8_t btn1;
  uint8_t btn2;
  uint8_t left_x;
  uint8_t left_y;
  uint8_t right_x;
  uint8_t right_y;
  uint8_t macro_active;
  uint8_t macro_button;
  float linear_x;
  float angular_z;
  uint32_t rx_ok_count;
  uint32_t rx_fail_count;
} ps2_control_state_t;

void Ps2Control_Init(void);
void Ps2Control_Update(void);
void Ps2Control_GetState(ps2_control_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
