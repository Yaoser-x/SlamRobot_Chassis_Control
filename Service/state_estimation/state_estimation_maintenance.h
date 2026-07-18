#ifndef STATE_ESTIMATION_MAINTENANCE_H
#define STATE_ESTIMATION_MAINTENANCE_H

#include <stdint.h>

#include "state_estimation_calibration_types.h"
#include "state_estimation_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** Unified result codes for non-trivial State Estimation operations. */
    typedef enum
    {
        STATE_ESTIMATION_RESULT_OK             = 0U,
        STATE_ESTIMATION_RESULT_BUSY           = 1U,
        STATE_ESTIMATION_RESULT_NULL_PARAMETER = 2U,
        STATE_ESTIMATION_RESULT_HARDWARE_ERROR = 3U
    } state_estimation_result_t;

    uint8_t StateEstimation_SetImuEnabled(uint8_t enabled);
    uint8_t StateEstimation_ProbeImu(void);
    uint8_t StateEstimation_ReinitializeImu(void);
    uint8_t StateEstimation_SetImuProfile(state_estimation_imu_profile_t profile);
    uint8_t StateEstimation_DiagnoseImu(state_estimation_imu_diagnostic_t *diagnostic);
    /** Advance the existing non-blocking IMU calibration state machine. */
    void    StateEstimation_ServiceImuCalibration(uint32_t now_ms, uint8_t stationary);
    uint8_t StateEstimation_ApplyImuCalibration(const imu_calibration_t *calibration);
    uint8_t StateEstimation_ClearImuCalibration(void);
    uint8_t StateEstimation_BeginImuCalibration(uint16_t samples, uint16_t interval_ms);
    uint8_t StateEstimation_GetImuCalibration(imu_calibration_t *calibration);

#ifdef __cplusplus
}
#endif

#endif /* STATE_ESTIMATION_MAINTENANCE_H */