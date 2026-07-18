#ifndef STATE_ESTIMATION_COMPOSITION_H
#define STATE_ESTIMATION_COMPOSITION_H

#include <stdint.h>

#include "state_estimation_calibration_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void StateEstimation_InitCalibrationCoordinator(void);
    uint8_t
    StateEstimation_ServiceCalibrationCoordinator(uint32_t                                           now_ms,
                                                  const state_estimation_calibration_motion_facts_t *motion_facts);

#ifdef __cplusplus
}
#endif

#endif /* STATE_ESTIMATION_COMPOSITION_H */
