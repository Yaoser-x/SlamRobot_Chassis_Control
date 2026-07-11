#ifndef DEBUG_MAINTENANCE_POLICY_H
#define DEBUG_MAINTENANCE_POLICY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG_MAINTENANCE_ARM_MS 60000UL

typedef struct
{
  uint32_t armed_at_ms;
  uint8_t armed;
} debug_maintenance_policy_t;

void DebugMaintenancePolicy_Init(debug_maintenance_policy_t *policy);
void DebugMaintenancePolicy_Arm(debug_maintenance_policy_t *policy, uint32_t now_ms);
void DebugMaintenancePolicy_Revoke(debug_maintenance_policy_t *policy);
uint8_t DebugMaintenancePolicy_Allowed(debug_maintenance_policy_t *policy,
                                       uint32_t now_ms,
                                       uint8_t release_requires_arm);
uint8_t DebugMaintenancePolicy_IsArmed(const debug_maintenance_policy_t *policy);

#ifdef __cplusplus
}
#endif

#endif
