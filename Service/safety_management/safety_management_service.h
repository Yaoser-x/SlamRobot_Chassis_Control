#ifndef SAFETY_MANAGEMENT_SERVICE_H
#define SAFETY_MANAGEMENT_SERVICE_H

#include <stdint.h>

#include "safety_management_config.h"
#include "safety_management_status.h"
#include "safety_management_types.h"

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
    void                  SafetyManagement_UpdateWithInput(const safety_management_input_t *input);
    uint32_t              SafetyManagement_GetStatus(safety_management_status_t *status);
    safety_clear_result_t SafetyManagement_ClearLatchedFaults(uint32_t mask, const safety_clear_input_t *input);
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
    /** Return whether explicitly armed diagnostic test mode may drive at its restricted limit. */
    uint8_t SafetyManagement_IsDiagnosticMotionAllowed(void);
    /** Evaluate a fact snapshot against the immutable product safety policy. */
    void SafetyManagement_EvaluateCapabilities(const safety_capability_input_t *input,
                                               safety_capability_permit_t      *capabilities);
    /** Publish App-evaluated capabilities and refresh the selected motion permit lease. */
    void SafetyManagement_ApplyCapabilityDecision(const safety_capability_permit_t *capabilities,
                                                  uint8_t                           normal_motion_permit,
                                                  uint32_t                          now_ms,
                                                  safety_runtime_state_t            runtime_state);
    /** Copy the current leased permit for independent Motor-cycle validation. */
    uint32_t SafetyManagement_GetMotionPermit(safety_motion_permit_t *permit);
    /** Apply the App coordinator's normal-motion permit and published runtime state. */
    void SafetyManagement_ApplyRuntimePermit(uint8_t permit, safety_runtime_state_t runtime_state);
    /** Apply the App coordinator's hardware-only permit for restricted diagnostic motion. */
    void SafetyManagement_ApplyDiagnosticPermit(uint8_t permit);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_MANAGEMENT_SERVICE_H */
