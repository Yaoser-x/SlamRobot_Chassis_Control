#include "debug_maintenance_policy.h"

void DebugMaintenancePolicy_Init(debug_maintenance_policy_t *policy)
{
  if (policy != 0)
  {
    policy->armed_at_ms = 0UL;
    policy->armed = 0U;
  }
}

void DebugMaintenancePolicy_Arm(debug_maintenance_policy_t *policy, uint32_t now_ms)
{
  if (policy != 0)
  {
    policy->armed_at_ms = now_ms;
    policy->armed = 1U;
  }
}

void DebugMaintenancePolicy_Revoke(debug_maintenance_policy_t *policy)
{
  if (policy != 0)
  {
    policy->armed_at_ms = 0UL;
    policy->armed = 0U;
  }
}

uint8_t DebugMaintenancePolicy_Allowed(debug_maintenance_policy_t *policy,
                                       uint32_t now_ms,
                                       uint8_t release_requires_arm)
{
  if (release_requires_arm == 0U)
  {
    return 1U;
  }
  if (policy == 0 || policy->armed == 0U)
  {
    return 0U;
  }
  if ((uint32_t)(now_ms - policy->armed_at_ms) > DEBUG_MAINTENANCE_ARM_MS)
  {
    DebugMaintenancePolicy_Revoke(policy);
    return 0U;
  }
  return 1U;
}

uint8_t DebugMaintenancePolicy_IsArmed(const debug_maintenance_policy_t *policy)
{
  return (policy != 0) ? policy->armed : 0U;
}
