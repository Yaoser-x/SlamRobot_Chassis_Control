#ifndef CHASSIS_RUNTIME_COORDINATOR_H
#define CHASSIS_RUNTIME_COORDINATOR_H

#include <stdint.h>

void     ChassisRuntimeCoordinator_Init(void);
void     ChassisRuntimeCoordinator_RunMotorCycle(uint32_t now_ms);
void     ChassisRuntimeCoordinator_CommitMotorCycle(uint32_t now_ms);
void     ChassisRuntimeCoordinator_RunSafetyCycle(uint32_t now_ms);
void     ChassisRuntimeCoordinator_CommitSafetyCycle(uint32_t now_ms);
uint8_t  ChassisRuntimeCoordinator_WatchdogFeedAllowed(uint32_t now_ms);
void     ChassisRuntimeCoordinator_CommitWatchdogFeed(void);
uint32_t ChassisRuntimeCoordinator_GetMotorCompletionGeneration(void);
uint32_t ChassisRuntimeCoordinator_GetSafetyCompletionGeneration(void);

#endif /* CHASSIS_RUNTIME_COORDINATOR_H */
