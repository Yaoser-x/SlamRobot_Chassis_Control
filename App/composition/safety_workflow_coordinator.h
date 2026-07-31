#ifndef SAFETY_WORKFLOW_COORDINATOR_H
#define SAFETY_WORKFLOW_COORDINATOR_H

#include <stdint.h>

#include "safety_management_status.h"

void                  AppSafetyWorkflow_SynchronizeCommandGate(void);
void                  AppSafetyWorkflow_SetEmergencyStop(uint8_t enabled);
void                  AppSafetyWorkflow_SetFaultStop(uint8_t enabled);
void                  AppSafetyWorkflow_LatchEncoderFeedbackFault(void);
safety_clear_result_t AppSafetyWorkflow_ClearLatchedFaults(uint32_t mask);
uint8_t               AppSafetyWorkflow_BeginMaintenance(void);
void                  AppSafetyWorkflow_EndMaintenance(void);

#endif /* SAFETY_WORKFLOW_COORDINATOR_H */
