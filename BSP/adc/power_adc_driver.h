#ifndef POWER_ADC_DRIVER_H
#define POWER_ADC_DRIVER_H

#include <stdint.h>

#include "motor_types.h"
#include "power_adc_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void PowerAdcDriver_Init(void);
    /** Configure the App-owned update period used for sample-rate diagnostics. */
    void PowerAdcDriver_SetUpdatePeriodMs(uint32_t period_ms);
    void PowerAdcDriver_Update(void);
    void PowerAdcDriver_GetState(power_adc_driver_state_t *state);
    void PowerAdcDriver_RequestCurrentZeroCalibration(void);
    void PowerAdcDriver_ApplyCurrentZeroCalibration(const uint16_t zero_raw[MOTOR_ID_COUNT]);
    /** Allow zero accumulation only while the chassis is confirmed stationary. */
    void PowerAdcDriver_SetCurrentZeroStationary(uint8_t stationary);

#ifdef __cplusplus
}
#endif

#endif
