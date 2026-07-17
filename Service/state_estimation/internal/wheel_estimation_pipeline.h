#ifndef WHEEL_ESTIMATION_PIPELINE_H
#define WHEEL_ESTIMATION_PIPELINE_H

#include "state_estimation_status.h"
#include "wheel_encoder_types.h"

/** @brief Reset the service-owned wheel estimation history. */
void WheelEstimationPipeline_Init(void);
/** @brief Convert raw timer facts into the hardware-independent wheel estimate. */
void WheelEstimationPipeline_Update(const wheel_encoder_sample_t    *sample,
                                    uint32_t                         now_ms,
                                    float                            wheel_radius_m,
                                    const int8_t                     encoder_dir[STATE_ESTIMATION_MOTOR_COUNT],
                                    state_estimation_wheel_status_t *status);
/** Aggregate per-motor validity across every enabled motor on each side. */
void WheelEstimationPipeline_AggregateSideValidity(const uint8_t speed_valid[STATE_ESTIMATION_MOTOR_COUNT],
                                                   uint8_t      *left_valid,
                                                   uint8_t      *right_valid);

#endif /* WHEEL_ESTIMATION_PIPELINE_H */
