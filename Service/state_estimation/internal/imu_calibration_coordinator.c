#include "imu_calibration_coordinator.h"

#include "imu_calibration_guard.h"
#include "state_estimation_service.h"
#include "state_estimation_maintenance.h"
#include "state_estimation_composition.h"

#define IMU_AUTOSAVE_RETRY_MS     1000U
#define IMU_AUTOSAVE_MAX_ATTEMPTS 3U

static imu_calibration_guard_t              calibration_gate;
static uint8_t                              first_save_needed;
static uint8_t                              save_pending;
static uint8_t                              save_attempts;
static uint32_t                             save_next_ms;
static uint8_t                              persist_imu_calibration;
static uint8_t                              persist_current_zero;
static state_estimation_calibration_ports_t calibration_ports;

void ImuCalibrationCoordinator_Init(const state_estimation_calibration_ports_t *ports,
                                    uint8_t                                     needs_first_save,
                                    uint8_t                                     persist_imu,
                                    uint8_t                                     persist_current)
{
    ImuCalibrationGuard_Init(&calibration_gate);
    first_save_needed       = needs_first_save;
    save_pending            = 0U;
    save_attempts           = 0U;
    save_next_ms            = 0U;
    persist_imu_calibration = (persist_imu != 0U) ? 1U : 0U;
    persist_current_zero    = (persist_current != 0U) ? 1U : 0U;
    calibration_ports       = (ports != 0) ? *ports : (state_estimation_calibration_ports_t){0};
    if (calibration_ports.set_current_zero_persistence != 0)
    {
        calibration_ports.set_current_zero_persistence(persist_current_zero);
    }
}

void ImuCalibrationCoordinator_ProcessSample(uint32_t now_ms)
{
    state_estimation_wheel_status_t wheel;
    state_estimation_imu_status_t   imu;
    int16_t                         output_permille[4] = {0};
    uint8_t                         enabled_mask       = 0U;
    uint8_t                         was_calibrated;
    uint8_t                         stationary;

    (void)StateEstimation_GetWheel(&wheel);
    (void)StateEstimation_GetImu(&imu);
    if (calibration_ports.get_motion_facts == 0
        || calibration_ports.get_motion_facts(output_permille, &enabled_mask) == 0U)
    {
        return;
    }
    was_calibrated = imu.gyro_calibrated;
    stationary     = ImuCalibrationGuard_Update(&calibration_gate,
                                            output_permille,
                                            wheel.speed_mps,
                                            wheel.speed_valid,
                                            enabled_mask,
                                            imu.body_accel_g,
                                            imu.gyro_corrected_dps,
                                            imu.sample_count);
    StateEstimation_ServiceImuCalibration(now_ms, stationary);
    (void)StateEstimation_GetImu(&imu);
    if (persist_imu_calibration != 0U && first_save_needed != 0U && was_calibrated == 0U && imu.gyro_calibrated != 0U)
    {
        first_save_needed = 0U;
        save_pending      = 1U;
        save_attempts     = 0U;
        save_next_ms      = now_ms;
    }
}

void ImuCalibrationCoordinator_ProcessPersistence(uint32_t now_ms)
{
    imu_calibration_t calibration;

    if (save_pending == 0U || (int32_t)(now_ms - save_next_ms) < 0)
    {
        return;
    }
    if (calibration_ports.begin_maintenance == 0 || calibration_ports.end_maintenance == 0
        || calibration_ports.persist == 0 || calibration_ports.begin_maintenance() == 0U)
    {
        return;
    }

    if (StateEstimation_GetImuCalibration(&calibration) != (uint8_t)STATE_ESTIMATION_RESULT_OK)
    {
        save_next_ms = now_ms + IMU_AUTOSAVE_RETRY_MS;
        calibration_ports.end_maintenance();
        return;
    }
    save_attempts++;
    if (calibration_ports.persist(&calibration, persist_current_zero) != 0U)
    {
        save_pending  = 0U;
        save_attempts = 0U;
    }
    else if (save_attempts >= IMU_AUTOSAVE_MAX_ATTEMPTS)
    {
        save_pending = 0U;
    }
    else
    {
        save_next_ms = now_ms + IMU_AUTOSAVE_RETRY_MS;
    }
    calibration_ports.end_maintenance();
}
