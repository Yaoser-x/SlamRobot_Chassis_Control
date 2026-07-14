#ifndef IMU_CALIBRATION_SERVICE_H
#define IMU_CALIBRATION_SERVICE_H

#include <stdint.h>

void ImuCalibrationService_Init(uint8_t first_save_needed);
void ImuCalibrationService_ProcessSample(uint32_t now_ms);
void ImuCalibrationService_ProcessPersistence(uint32_t now_ms);

#endif
