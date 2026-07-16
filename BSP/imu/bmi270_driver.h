#ifndef BMI270_DRIVER_H
#define BMI270_DRIVER_H

#include "bmi270_bus.h"
#include "bmi270_types.h"
#include "bmi270_profile.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief Initialize the BMI270 device owner and raw-sample queue. */
    void    Bmi270Driver_Init(void);
    uint8_t Bmi270Driver_SetEnabled(uint8_t enabled);
    uint8_t Bmi270Driver_SetProfile(imu_bmi270_profile_id_t profile);
    uint8_t Bmi270Driver_ProbeNow(void);
    uint8_t Bmi270Driver_ConfigNow(void);
    /** @brief Read FIFO or registers and enqueue raw device samples. */
    uint8_t Bmi270Driver_Update(void);
    /** @brief Pop one raw sample for the State Estimation service. */
    uint8_t Bmi270Driver_TakeSample(bmi270_sample_t *sample);
    void    Bmi270Driver_OnDataReadyFromIsr(void);
    uint8_t Bmi270Driver_Diagnose(imu_bmi270_diag_t *diag);
    void    Bmi270Driver_GetState(bmi270_driver_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* BMI270_DRIVER_H */
