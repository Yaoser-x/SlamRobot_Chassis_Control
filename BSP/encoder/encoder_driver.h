#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <stdint.h>

#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_SIDE_CONSISTENCY_LEFT_SPEED     (1UL << 0)
#define ENCODER_SIDE_CONSISTENCY_LEFT_COUNT     (1UL << 1)
#define ENCODER_SIDE_CONSISTENCY_LEFT_DIRECTION (1UL << 2)
#define ENCODER_SIDE_CONSISTENCY_RIGHT_SPEED    (1UL << 3)
#define ENCODER_SIDE_CONSISTENCY_RIGHT_COUNT    (1UL << 4)
#define ENCODER_SIDE_CONSISTENCY_RIGHT_DIRECTION (1UL << 5)

typedef struct
{
  int32_t count[MOTOR_ID_COUNT];
  int32_t delta[MOTOR_ID_COUNT];
  float speed_mps[MOTOR_ID_COUNT];
  uint8_t speed_valid[MOTOR_ID_COUNT];
  uint8_t reject_streak[MOTOR_ID_COUNT];
  uint16_t window_rebuild_count[MOTOR_ID_COUNT];
  uint16_t anomaly_count[MOTOR_ID_COUNT];
  uint8_t consecutive_anomalies[MOTOR_ID_COUNT];
  int32_t left_count;
  int32_t right_count;
  int32_t left_delta;
  int32_t right_delta;
  float left_speed_mps;
  float right_speed_mps;
  uint8_t left_speed_valid;
  uint8_t right_speed_valid;
  uint8_t speed_valid_all;
  uint32_t side_consistency_flags;
  uint32_t last_update_ms;
} encoder_state_t;

void EncoderDriver_Init(void);
void EncoderDriver_Update(uint32_t now_ms);
void EncoderDriver_GetState(encoder_state_t *state);
float EncoderDriver_GetCountsPerRev(void);
float EncoderDriver_GetMotorSpeedMps(motor_id_t motor);
int32_t EncoderDriver_DiffCount(uint32_t now, uint32_t last, uint32_t period);

#ifdef __cplusplus
}
#endif

#endif
