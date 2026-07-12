#include "debug_maintenance_policy.h"

#include <stdio.h>
#include <stdlib.h>

static void require_int(int condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

int main(void)
{
    debug_maintenance_policy_t policy;

    DebugMaintenancePolicy_Init(&policy);
    require_int(DebugMaintenancePolicy_Allowed(&policy, 100U, 0U) != 0U, "debug build does not require arm");
    require_int(DebugMaintenancePolicy_Allowed(&policy, 100U, 1U) == 0U, "release build rejects before arm");

    DebugMaintenancePolicy_Arm(&policy, 100U);
    require_int(DebugMaintenancePolicy_Allowed(&policy, 60100U, 1U) != 0U,
                "release arm is valid at 60 second boundary");
    require_int(DebugMaintenancePolicy_Allowed(&policy, 60101U, 1U) == 0U, "release arm expires after 60 seconds");
    require_int(DebugMaintenancePolicy_IsArmed(&policy) == 0U, "expired release arm is revoked");

    DebugMaintenancePolicy_Arm(&policy, 200U);
    DebugMaintenancePolicy_Revoke(&policy);
    require_int(DebugMaintenancePolicy_Allowed(&policy, 201U, 1U) == 0U, "maint off revokes release authorization");

    DebugMaintenancePolicy_Arm(&policy, 0xFFFFFFF0UL);
    require_int(DebugMaintenancePolicy_Allowed(&policy, 0x00000010UL, 1U) != 0U, "authorization handles tick wrap");

    (void)printf("PASS: debug maintenance policy host tests\n");
    return 0;
}
