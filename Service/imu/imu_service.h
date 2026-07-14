#ifndef IMU_SERVICE_H
#define IMU_SERVICE_H

#include "imu_bmi270.h"

typedef imu_bmi270_state_t imu_service_snapshot_t;

void    ImuService_Init(void);
uint8_t ImuService_RunCycle(void);
void    ImuService_OnDataReadyFromIsr(void);
void    ImuService_ServiceCalibration(uint32_t now_ms, uint8_t stationary);
uint8_t ImuService_ApplyCalibration(const imu_bmi270_calibration_t *calibration);
void    ImuService_GetCalibration(imu_bmi270_calibration_t *calibration);
void    ImuService_GetSnapshot(imu_service_snapshot_t *snapshot);

#endif
