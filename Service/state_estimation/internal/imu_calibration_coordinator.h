#ifndef IMU_CALIBRATION_COORDINATOR_H
#define IMU_CALIBRATION_COORDINATOR_H

#include <stdint.h>
#include "state_estimation_calibration_types.h"

/** Initialize IMU calibration and persistence orchestration with product policy. */
void ImuCalibrationCoordinator_Init(const state_estimation_calibration_ports_t *ports,
                                    uint8_t first_save_needed,
                                    uint8_t persist_imu_calibration,
                                    uint8_t persist_current_zero);
/** Process one IMU sample against the current stationary facts. */
void ImuCalibrationCoordinator_ProcessSample(uint32_t now_ms);
/** Service the bounded calibration persistence retry state machine. */
void ImuCalibrationCoordinator_ProcessPersistence(uint32_t now_ms);

#endif /* APP_IMU_CALIBRATION_H */
