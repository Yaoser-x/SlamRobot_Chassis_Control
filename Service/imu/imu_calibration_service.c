#include "imu_calibration_service.h"

#include "chassis_layout.h"
#include "chassis_maintenance_service.h"
#include "current_sensor_service.h"
#include "encoder_service.h"
#include "imu_calibration_gate.h"
#include "imu_service.h"
#include "motor_driver.h"
#include "param_persistence.h"
#include "param_service.h"

#define IMU_AUTOSAVE_RETRY_MS     1000U
#define IMU_AUTOSAVE_MAX_ATTEMPTS 3U

static imu_calibration_gate_t calibration_gate;
static uint8_t                first_save_needed;
static uint8_t                save_pending;
static uint8_t                save_attempts;
static uint32_t               save_next_ms;

void ImuCalibrationService_Init(uint8_t needs_first_save)
{
    ImuCalibrationGate_Init(&calibration_gate);
    first_save_needed = needs_first_save;
    save_pending      = 0U;
    save_attempts     = 0U;
    save_next_ms      = 0U;
}

void ImuCalibrationService_ProcessSample(uint32_t now_ms)
{
    encoder_service_snapshot_t encoder;
    motor_driver_state_t       motor;
    imu_service_snapshot_t     imu;
    uint8_t                    enabled_mask = 0U;
    uint8_t                    was_calibrated;
    uint8_t                    stationary;

    EncoderService_GetSnapshot(&encoder);
    MotorDriver_GetState(&motor);
    ImuService_GetSnapshot(&imu);
    was_calibrated = imu.gyro_calibrated;
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        if (ChassisLayout_MotorEnabled((motor_id_t)index) != 0U)
        {
            enabled_mask |= (uint8_t)(1U << index);
        }
    }
    stationary = ImuCalibrationGate_Update(&calibration_gate,
                                           motor.effective_pwm,
                                           encoder.speed_mps,
                                           encoder.speed_valid,
                                           enabled_mask,
                                           imu.body_accel_g,
                                           imu.gyro_corrected_dps,
                                           imu.sample_count);
    ImuService_ServiceCalibration(now_ms, stationary);
    ImuService_GetSnapshot(&imu);
    if (first_save_needed != 0U && was_calibrated == 0U && imu.gyro_calibrated != 0U)
    {
        first_save_needed = 0U;
        save_pending      = 1U;
        save_attempts     = 0U;
        save_next_ms      = now_ms;
    }
}

void ImuCalibrationService_ProcessPersistence(uint32_t now_ms)
{
    current_sensor_snapshot_t current;
    flash_param_bundle_t      bundle;

    if (save_pending == 0U || (int32_t)(now_ms - save_next_ms) < 0)
    {
        return;
    }
    if (ChassisMaintenanceService_Begin() != CHASSIS_MAINTENANCE_SERVICE_OK)
    {
        return;
    }

    ParamService_Get(&bundle.params);
    CurrentSensorService_GetSnapshot(&current);
    if (current.current_zero_valid != 0U)
    {
        for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
        {
            bundle.params.current_zero_raw[index] = current.current_zero_raw[index];
        }
        bundle.params.current_zero_valid = 1U;
    }
    ImuService_GetCalibration(&bundle.imu_calibration);
    save_attempts++;
    if (ParamPersistence_Save(&bundle) == FLASH_PARAM_STATUS_OK)
    {
        (void)ParamService_Set(&bundle.params);
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
    ChassisMaintenanceService_End();
}
