#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <stdint.h>

#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  int32_t count[MOTOR_ID_COUNT];
  int32_t delta[MOTOR_ID_COUNT];
  float speed_mps[MOTOR_ID_COUNT];
  uint8_t speed_valid[MOTOR_ID_COUNT];
  int32_t left_count;
  int32_t right_count;
  int32_t left_delta;
  int32_t right_delta;
  float left_speed_mps;
  float right_speed_mps;
  uint8_t left_speed_valid;
  uint8_t right_speed_valid;
  uint8_t speed_valid_all;
  uint32_t last_update_ms;
} encoder_state_t;

void EncoderDriver_Init(void);
void EncoderDriver_Update(uint32_t now_ms);
void EncoderDriver_GetState(encoder_state_t *state);
float EncoderDriver_GetCountsPerRev(void);
int32_t EncoderDriver_DiffCount(uint32_t now, uint32_t last, uint32_t period);

#ifdef __cplusplus
}
#endif

#endif
