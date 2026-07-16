#ifndef POWER_MANAGEMENT_SERVICE_H
#define POWER_MANAGEMENT_SERVICE_H

#include <stdint.h>

#include "power_management_config.h"
#include "power_management_status.h"

#ifdef __cplusplus
extern "C"
{
#endif

    uint8_t PowerManagement_Init(const power_management_config_t *config);
    uint8_t PowerManagement_IsInitialized(void);
    /** Update ADC facts once from the existing 20 ms Safety task. */
    void PowerManagement_Update(void);
    /** Update whether current-zero accumulation is allowed from State and Motor facts. */
    void     PowerManagement_UpdateStationary(void);
    void     PowerManagement_RequestCurrentZeroCalibration(void);
    void     PowerManagement_ApplyCurrentZeroCalibration(const uint16_t zero_raw[POWER_MEASUREMENT_MOTOR_COUNT]);
    uint32_t PowerManagement_GetStatus(power_management_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* POWER_MANAGEMENT_SERVICE_H */
