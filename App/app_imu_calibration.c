#include "app_imu_calibration.h"

#include "imu_calibration_guard.h"
#include "motion_control_service.h"
#include "parameter_management_service.h"
#include "power_management_service.h"
#include "state_estimation_service.h"

#define IMU_AUTOSAVE_RETRY_MS     1000U
#define IMU_AUTOSAVE_MAX_ATTEMPTS 3U

static imu_calibration_guard_t calibration_gate;
static uint8_t                 first_save_needed;
static uint8_t                 save_pending;
static uint8_t                 save_attempts;
static uint32_t                save_next_ms;
static uint8_t                 persist_imu_calibration;
static uint8_t                 persist_current_zero;

void AppImuCalibration_Init(uint8_t needs_first_save, uint8_t persist_imu, uint8_t persist_current)
{
    ImuCalibrationGuard_Init(&calibration_gate);
    first_save_needed       = needs_first_save;
    save_pending            = 0U;
    save_attempts           = 0U;
    save_next_ms            = 0U;
    persist_imu_calibration = (persist_imu != 0U) ? 1U : 0U;
    persist_current_zero    = (persist_current != 0U) ? 1U : 0U;
    ParameterManagement_SetCurrentZeroPersistence(persist_current_zero);
}

void AppImuCalibration_ProcessSample(uint32_t now_ms)
{
    state_estimation_wheel_status_t wheel;
    state_estimation_imu_status_t   imu;
    motion_control_status_t         motion;
    uint8_t                         was_calibrated;
    uint8_t                         stationary;

    (void)StateEstimation_GetWheel(&wheel);
    (void)StateEstimation_GetImu(&imu);
    (void)MotionControl_GetStatus(&motion);
    was_calibrated = imu.gyro_calibrated;
    stationary     = ImuCalibrationGuard_Update(&calibration_gate,
                                            motion.motor_effective_output_permille,
                                            wheel.speed_mps,
                                            wheel.speed_valid,
                                            motion.motor_enabled_mask,
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

void AppImuCalibration_ProcessPersistence(uint32_t now_ms)
{
    power_management_status_t current;
    param_model_t             params;
    imu_bmi270_calibration_t  calibration;

    if (save_pending == 0U || (int32_t)(now_ms - save_next_ms) < 0)
    {
        return;
    }
    if (MotionControl_BeginMaintenance() != MOTION_CONTROL_MAINTENANCE_OK)
    {
        return;
    }

    (void)ParameterManagement_GetSnapshot(&params);
    (void)PowerManagement_GetStatus(&current);
    if (persist_current_zero != 0U && current.current_zero_valid != 0U)
    {
        for (uint8_t index = 0U; index < MOTION_CONTROL_MOTOR_COUNT; ++index)
        {
            params.current_zero_raw[index] = current.current_zero_raw[index];
        }
        params.current_zero_valid = 1U;
    }
    else if (persist_current_zero == 0U)
    {
        for (uint8_t index = 0U; index < MOTION_CONTROL_MOTOR_COUNT; ++index)
        {
            params.current_zero_raw[index] = 0U;
        }
        params.current_zero_valid = 0U;
    }
    StateEstimation_GetImuCalibration(&calibration);
    ParameterManagement_SetImuCalibration(&calibration);
    if (persist_current_zero != 0U)
    {
        (void)ParameterManagement_Set(&params);
    }
    save_attempts++;
    if (ParameterManagement_Save() != 0U)
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
    MotionControl_EndMaintenance();
}
