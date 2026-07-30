#ifndef CHASSIS_RUNTIME_COORDINATOR_H
#define CHASSIS_RUNTIME_COORDINATOR_H

#include <stdint.h>

void     ChassisRuntimeCoordinator_Init(void);
void     ChassisRuntimeCoordinator_RunMotorCycle(uint32_t now_ms);
void     ChassisRuntimeCoordinator_RunSafetyCycle(uint32_t now_ms);
uint8_t  ChassisRuntimeCoordinator_ShouldFeedWatchdog(void);
uint32_t ChassisRuntimeCoordinator_GetMotorCompletionGeneration(void);
uint32_t ChassisRuntimeCoordinator_GetSafetyCompletionGeneration(void);

#endif /* CHASSIS_RUNTIME_COORDINATOR_H */
