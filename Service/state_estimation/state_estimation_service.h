#ifndef STATE_ESTIMATION_SERVICE_H
#define STATE_ESTIMATION_SERVICE_H

#include <stdint.h>

#include "state_estimation_config.h"
#include "state_estimation_status.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** Initialize the wheel and IMU estimation owner without merging their schedulers. */
    uint8_t StateEstimation_Init(const state_estimation_config_t *config);
    uint8_t StateEstimation_IsInitialized(void);
    /** Run the 10 ms wheel-count and wheel-speed chain. */
    void StateEstimation_UpdateWheel(uint32_t now_ms);
    /** Run one DRDY-triggered or 10 ms timeout IMU chain iteration. */
    uint8_t StateEstimation_RunImuCycle(void);
    /** Forward the IMU data-ready event from ISR context. */
    void     StateEstimation_OnImuDataReadyFromIsr(void);
    uint32_t StateEstimation_GetWheel(state_estimation_wheel_status_t *status);
    /** Acknowledge physical Host delivery of a STATUS carrying an anomaly generation. */
    uint8_t  StateEstimation_AcknowledgeWheelAnomalyDelivery(uint32_t generation);
    uint32_t StateEstimation_GetImu(state_estimation_imu_status_t *status);
    uint32_t StateEstimation_GetStatus(uint32_t now_ms, state_estimation_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* STATE_ESTIMATION_SERVICE_H */
