#ifndef WHEEL_ESTIMATION_TYPES_H
#define WHEEL_ESTIMATION_TYPES_H

#include <stdint.h>

#define WHEEL_ESTIMATION_MOTOR_COUNT             4U

#define ENCODER_SIDE_CONSISTENCY_LEFT_SPEED      (1UL << 0)
#define ENCODER_SIDE_CONSISTENCY_LEFT_COUNT      (1UL << 1)
#define ENCODER_SIDE_CONSISTENCY_LEFT_DIRECTION  (1UL << 2)
#define ENCODER_SIDE_CONSISTENCY_RIGHT_SPEED     (1UL << 3)
#define ENCODER_SIDE_CONSISTENCY_RIGHT_COUNT     (1UL << 4)
#define ENCODER_SIDE_CONSISTENCY_RIGHT_DIRECTION (1UL << 5)

/** @brief Hardware-independent wheel-count and wheel-speed estimate. */
typedef struct
{
    int32_t  count[WHEEL_ESTIMATION_MOTOR_COUNT];
    int32_t  delta[WHEEL_ESTIMATION_MOTOR_COUNT];
    float    speed_mps[WHEEL_ESTIMATION_MOTOR_COUNT];
    uint8_t  speed_valid[WHEEL_ESTIMATION_MOTOR_COUNT];
    uint8_t  reject_streak[WHEEL_ESTIMATION_MOTOR_COUNT];
    uint16_t window_rebuild_count[WHEEL_ESTIMATION_MOTOR_COUNT];
    uint16_t anomaly_count[WHEEL_ESTIMATION_MOTOR_COUNT];
    uint8_t  consecutive_anomalies[WHEEL_ESTIMATION_MOTOR_COUNT];
    int32_t  left_count;
    int32_t  right_count;
    int32_t  left_delta;
    int32_t  right_delta;
    float    left_speed_mps;
    float    right_speed_mps;
    uint8_t  left_speed_valid;
    uint8_t  right_speed_valid;
    uint8_t  speed_valid_all;
    uint32_t side_consistency_flags;
    uint32_t last_update_ms;
} wheel_encoder_state_t;

#endif /* WHEEL_ESTIMATION_TYPES_H */
