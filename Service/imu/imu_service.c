#include "imu_service.h"

void ImuService_Init(void)
{
    ImuBmi270_Init();
}

uint8_t ImuService_RunCycle(void)
{
    return ImuBmi270_Update();
}

void ImuService_OnDataReadyFromIsr(void)
{
    ImuBmi270_OnDataReadyFromIsr();
}

void ImuService_ServiceCalibration(uint32_t now_ms, uint8_t stationary)
{
    ImuBmi270_ServiceCalibration(now_ms, stationary);
}

uint8_t ImuService_ApplyCalibration(const imu_bmi270_calibration_t *calibration)
{
    return ImuBmi270_ApplyCalibration(calibration);
}

void ImuService_GetCalibration(imu_bmi270_calibration_t *calibration)
{
    ImuBmi270_GetCalibration(calibration);
}

void ImuService_GetSnapshot(imu_service_snapshot_t *snapshot)
{
    ImuBmi270_GetState(snapshot);
}
