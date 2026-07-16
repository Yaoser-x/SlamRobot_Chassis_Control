#ifndef IMU_BMI270_H
#define IMU_BMI270_H

#include <stdint.h>

#include "bmi270_bus.h"
#include "bmi270_calibration.h"
#include "bmi270_types.h"
#include "imu_bmi270_profile.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void    Bmi270Driver_Init(void);
    uint8_t Bmi270Driver_SetEnabled(uint8_t enabled);
    uint8_t Bmi270Driver_SetProfile(imu_bmi270_profile_id_t profile);
    uint8_t Bmi270Driver_ProbeNow(void);
    uint8_t Bmi270Driver_ConfigNow(void);
    uint8_t Bmi270Driver_Update(void);
    void    Bmi270Driver_OnDataReadyFromIsr(void);
    uint8_t Bmi270Driver_Diagnose(imu_bmi270_diag_t *diag);
    uint8_t Bmi270Driver_RawFrameHasSignal(const int16_t accel_raw[3], const int16_t gyro_raw[3]);
    uint8_t Bmi270Driver_GyroCalSpanWithinLimit(const float min_dps[3],
                                                const float max_dps[3],
                                                float       max_span_dps,
                                                uint8_t    *axis);
    uint8_t Bmi270Driver_AutoCalDue(uint8_t  enabled,
                                    uint8_t  online,
                                    uint8_t  calibrated,
                                    uint8_t  attempts,
                                    uint8_t  max_attempts,
                                    uint32_t now_ms,
                                    uint32_t next_ms);
    uint8_t Bmi270Driver_CalibrateGyro(uint16_t samples, uint16_t delay_ms);
    void    Bmi270Driver_ServiceCalibration(uint32_t now_ms, uint8_t stationary);
    void    Bmi270Driver_ClearCalibration(void);
    void    Bmi270Driver_ApplyGyroBias(const float bias_dps[3]);
    uint8_t Bmi270Driver_ApplyCalibration(const bmi270_calibration_t *calibration);
    void    Bmi270Driver_GetCalibration(bmi270_calibration_t *calibration);
    void    Bmi270Driver_GetState(bmi270_driver_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
