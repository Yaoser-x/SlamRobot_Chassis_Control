#ifndef IMU_CALIBRATION_COORDINATOR_H
#define IMU_CALIBRATION_COORDINATOR_H

#include <stdint.h>
#include "state_estimation_calibration_types.h"

/** Initialize the State Estimation-owned stationary guard. */
void ImuCalibrationCoordinator_Init(void);
/** Process one IMU sample against the current stationary facts. */
uint8_t ImuCalibrationCoordinator_ProcessSample(uint32_t                                           now_ms,
                                                const state_estimation_calibration_motion_facts_t *motion_facts);

#endif /* APP_IMU_CALIBRATION_H */
