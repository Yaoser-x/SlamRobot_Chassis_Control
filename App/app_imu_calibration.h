#ifndef APP_IMU_CALIBRATION_H
#define APP_IMU_CALIBRATION_H

#include <stdint.h>

/** Initialize IMU calibration and persistence orchestration with product policy. */
void AppImuCalibration_Init(uint8_t first_save_needed, uint8_t persist_imu_calibration, uint8_t persist_current_zero);
/** Process one IMU sample against the current stationary facts. */
void AppImuCalibration_ProcessSample(uint32_t now_ms);
/** Service the bounded calibration persistence retry state machine. */
void AppImuCalibration_ProcessPersistence(uint32_t now_ms);

#endif /* APP_IMU_CALIBRATION_H */
