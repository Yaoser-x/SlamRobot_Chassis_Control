#ifndef IMU_BMI270_H
#define IMU_BMI270_H

#include <stdint.h>

#include "bmi270_bus.h"
#include "imu_calibration.h"
#include "imu_estimation_types.h"
#include "imu_bmi270_profile.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef imu_estimation_t imu_bmi270_state_t;

    void    ImuBmi270_Init(void);
    uint8_t ImuBmi270_SetEnabled(uint8_t enabled);
    uint8_t ImuBmi270_SetProfile(imu_bmi270_profile_id_t profile);
    uint8_t ImuBmi270_ProbeNow(void);
    uint8_t ImuBmi270_ConfigNow(void);
    uint8_t ImuBmi270_Update(void);
    void    ImuBmi270_OnDataReadyFromIsr(void);
    uint8_t ImuBmi270_Diagnose(imu_bmi270_diag_t *diag);
    uint8_t ImuBmi270_RawFrameHasSignal(const int16_t accel_raw[3], const int16_t gyro_raw[3]);
    uint8_t
    ImuBmi270_GyroCalSpanWithinLimit(const float min_dps[3], const float max_dps[3], float max_span_dps, uint8_t *axis);
    uint8_t ImuBmi270_AutoCalDue(uint8_t  enabled,
                                 uint8_t  online,
                                 uint8_t  calibrated,
                                 uint8_t  attempts,
                                 uint8_t  max_attempts,
                                 uint32_t now_ms,
                                 uint32_t next_ms);
    uint8_t ImuBmi270_CalibrateGyro(uint16_t samples, uint16_t delay_ms);
    void    ImuBmi270_ServiceCalibration(uint32_t now_ms, uint8_t stationary);
    void    ImuBmi270_ClearCalibration(void);
    void    ImuBmi270_ApplyGyroBias(const float bias_dps[3]);
    uint8_t ImuBmi270_ApplyCalibration(const imu_bmi270_calibration_t *calibration);
    void    ImuBmi270_GetCalibration(imu_bmi270_calibration_t *calibration);
    void    ImuBmi270_GetState(imu_bmi270_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
