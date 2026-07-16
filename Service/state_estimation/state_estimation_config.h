#ifndef STATE_ESTIMATION_CONFIG_H
#define STATE_ESTIMATION_CONFIG_H

#include <stdint.h>

/** @brief Product configuration consumed by State Estimation. */
typedef struct
{
    uint32_t wheel_feedback_timeout_ms;
    uint32_t imu_fresh_timeout_ms;
} state_estimation_config_t;

#endif
