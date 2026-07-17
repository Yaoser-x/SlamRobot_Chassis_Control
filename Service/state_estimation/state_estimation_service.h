#ifndef STATE_ESTIMATION_SERVICE_H
#define STATE_ESTIMATION_SERVICE_H

#include <stdint.h>

#include "state_estimation_calibration_types.h"
#include "state_estimation_config.h"
#include "state_estimation_status.h"

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

    /** Initialize the wheel and IMU estimation owner without merging their schedulers. */
    uint8_t StateEstimation_Init(const state_estimation_config_t *config);
    uint8_t StateEstimation_IsInitialized(void);
    uint8_t StateEstimation_SetImuEnabled(uint8_t enabled);
    uint8_t StateEstimation_ProbeImu(void);
    uint8_t StateEstimation_ReinitializeImu(void);
    uint8_t StateEstimation_SetImuProfile(state_estimation_imu_profile_t profile);
    uint8_t StateEstimation_DiagnoseImu(state_estimation_imu_diagnostic_t *diagnostic);
    /** Run the 10 ms wheel-count and wheel-speed chain. */
    void StateEstimation_UpdateWheel(uint32_t now_ms);
    /** Run one DRDY-triggered or 10 ms timeout IMU chain iteration. */
    uint8_t StateEstimation_RunImuCycle(void);
    /** Forward the IMU data-ready event from ISR context. */
    void StateEstimation_OnImuDataReadyFromIsr(void);
    /** Advance the existing non-blocking IMU calibration state machine. */
    void     StateEstimation_ServiceImuCalibration(uint32_t now_ms, uint8_t stationary);
    uint8_t  StateEstimation_ApplyImuCalibration(const imu_calibration_t *calibration);
    uint8_t  StateEstimation_ClearImuCalibration(void);
    uint8_t  StateEstimation_BeginImuCalibration(uint16_t samples, uint16_t interval_ms);
    uint8_t  StateEstimation_GetImuCalibration(imu_calibration_t *calibration);
    void     StateEstimation_InitCalibrationCoordinator(const state_estimation_calibration_ports_t *ports,
                                                        uint8_t                                     first_save_needed,
                                                        uint8_t                                     persist_imu_calibration,
                                                        uint8_t                                     persist_current_zero);
    void     StateEstimation_ServiceCalibrationCoordinator(uint32_t now_ms);
    void     StateEstimation_ServiceCalibrationPersistence(uint32_t now_ms);
    uint32_t StateEstimation_GetWheel(state_estimation_wheel_status_t *status);
    uint32_t StateEstimation_GetImu(state_estimation_imu_status_t *status);
    uint32_t StateEstimation_GetStatus(uint32_t now_ms, state_estimation_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* STATE_ESTIMATION_SERVICE_H */
