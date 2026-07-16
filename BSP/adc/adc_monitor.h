#ifndef ADC_MONITOR_H
#define ADC_MONITOR_H

#include <stdint.h>

#include "motor_types.h"
#include "power_measurement_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef power_measurement_t adc_monitor_state_t;

    void AdcMonitor_Init(void);
    /** Configure the App-owned update period used for sample-rate diagnostics. */
    void AdcMonitor_SetUpdatePeriodMs(uint32_t period_ms);
    void AdcMonitor_Update(void);
    void AdcMonitor_GetState(adc_monitor_state_t *state);
    void AdcMonitor_RequestCurrentZeroCalibration(void);
    void AdcMonitor_ApplyCurrentZeroCalibration(const uint16_t zero_raw[MOTOR_ID_COUNT]);
    /** Allow zero accumulation only while the chassis is confirmed stationary. */
    void AdcMonitor_SetCurrentZeroStationary(uint8_t stationary);

#ifdef __cplusplus
}
#endif

#endif
