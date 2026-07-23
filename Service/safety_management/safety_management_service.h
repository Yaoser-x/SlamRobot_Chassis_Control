#ifndef SAFETY_MANAGEMENT_SERVICE_H
#define SAFETY_MANAGEMENT_SERVICE_H

#include <stdint.h>

#include "safety_management_config.h"
#include "safety_management_status.h"

#ifdef __cplusplus
extern "C"
{
#endif

    uint8_t SafetyManagement_ValidateConfig(const safety_management_config_t *config);
    uint8_t SafetyManagement_Init(const safety_management_config_t *config);
    /** Refresh Parameter-owned current thresholds before the next safety cycle. */
    void                  SafetyManagement_SetCurrentThresholds(const float observe_a[SAFETY_MANAGEMENT_MOTOR_COUNT],
                                                                const float fault_a[SAFETY_MANAGEMENT_MOTOR_COUNT],
                                                                uint16_t    debounce_ms);
    uint8_t               SafetyManagement_IsInitialized(void);
    void                  SafetyManagement_Update(void);
    uint32_t              SafetyManagement_GetStatus(safety_management_status_t *status);
    safety_clear_result_t SafetyManagement_ClearLatchedFaults(uint32_t mask);
    uint8_t               SafetyManagement_HasLatchedFault(void);
    void                  SafetyManagement_LatchEncoderFeedbackFault(void);
    void                  SafetyManagement_SetEmergencyStop(uint8_t enabled);
    void                  SafetyManagement_SetFaultStop(uint8_t enabled);
    uint8_t               SafetyManagement_BeginMaintenance(void);
    void                  SafetyManagement_EndMaintenance(void);
    uint8_t               SafetyManagement_IsEmergencyStop(void);
    uint8_t               SafetyManagement_IsFaultStop(void);
    uint8_t               SafetyManagement_IsMaintenanceLocked(void);
    uint8_t               SafetyManagement_IsMotionAllowed(void);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_MANAGEMENT_SERVICE_H */
