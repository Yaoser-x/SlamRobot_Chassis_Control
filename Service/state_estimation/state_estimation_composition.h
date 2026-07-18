#ifndef STATE_ESTIMATION_COMPOSITION_H
#define STATE_ESTIMATION_COMPOSITION_H

#include <stdint.h>

#include "state_estimation_calibration_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void StateEstimation_InitCalibrationCoordinator(const state_estimation_calibration_ports_t *ports,
                                                     uint8_t                                     first_save_needed,
                                                     uint8_t                                     persist_imu_calibration,
                                                     uint8_t                                     persist_current_zero);
    void StateEstimation_ServiceCalibrationCoordinator(uint32_t now_ms);
    void StateEstimation_ServiceCalibrationPersistence(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* STATE_ESTIMATION_COMPOSITION_H */