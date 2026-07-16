#ifndef STATE_ESTIMATION_STATUS_H
#define STATE_ESTIMATION_STATUS_H

#include <stdint.h>

#include "imu_estimation_types.h"
#include "wheel_estimation_types.h"

typedef wheel_estimation_t state_estimation_wheel_status_t;
typedef imu_estimation_t   state_estimation_imu_status_t;

/** @brief Wheel and IMU snapshots with independent freshness and generation. */
typedef struct
{
    state_estimation_wheel_status_t wheel;
    state_estimation_imu_status_t   imu;
    uint32_t                        wheel_generation;
    uint32_t                        imu_generation;
    uint8_t                         wheel_fresh;
    uint8_t                         imu_fresh;
} state_estimation_status_t;

#endif /* STATE_ESTIMATION_STATUS_H */
