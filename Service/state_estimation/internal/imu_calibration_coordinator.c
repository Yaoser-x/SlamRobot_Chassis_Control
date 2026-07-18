#include "imu_calibration_coordinator.h"

#include "imu_calibration_guard.h"
#include "state_estimation_maintenance.h"
#include "state_estimation_service.h"

static imu_calibration_guard_t calibration_gate;

void ImuCalibrationCoordinator_Init(void)
{
    ImuCalibrationGuard_Init(&calibration_gate);
}

uint8_t ImuCalibrationCoordinator_ProcessSample(uint32_t                                           now_ms,
                                                const state_estimation_calibration_motion_facts_t *motion_facts)
{
    state_estimation_wheel_status_t wheel;
    state_estimation_imu_status_t   imu;
    uint8_t                         was_calibrated;
    uint8_t                         stationary;

    if (motion_facts == 0)
    {
        return 0U;
    }
    (void)StateEstimation_GetWheel(&wheel);
    (void)StateEstimation_GetImu(&imu);
    was_calibrated = imu.gyro_calibrated;
    stationary     = ImuCalibrationGuard_Update(&calibration_gate,
                                            motion_facts->output_permille,
                                            wheel.speed_mps,
                                            wheel.speed_valid,
                                            motion_facts->enabled_mask,
                                            imu.body_accel_g,
                                            imu.gyro_corrected_dps,
                                            imu.sample_count);
    StateEstimation_ServiceImuCalibration(now_ms, stationary);
    (void)StateEstimation_GetImu(&imu);
    return (was_calibrated == 0U && imu.gyro_calibrated != 0U) ? 1U : 0U;
}
