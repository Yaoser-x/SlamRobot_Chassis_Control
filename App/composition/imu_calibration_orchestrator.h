#ifndef IMU_CALIBRATION_ORCHESTRATOR_H
#define IMU_CALIBRATION_ORCHESTRATOR_H

#include <stdint.h>

/** Initialize App-owned IMU calibration persistence policy. */
void ImuCalibrationOrchestrator_Init(uint8_t persist_imu_calibration, uint8_t persist_current_zero);
/** Collect Motion facts and advance one State Estimation calibration sample. */
void ImuCalibrationOrchestrator_ProcessSample(uint32_t now_ms);
/** Service bounded Flash persistence retries from the Safety task. */
void ImuCalibrationOrchestrator_ProcessPersistence(uint32_t now_ms);

#endif /* IMU_CALIBRATION_ORCHESTRATOR_H */
