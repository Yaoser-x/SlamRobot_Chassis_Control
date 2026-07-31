#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "safety_motion_permit_guard.h"

static void require_int(int condition, const char *message)
{
    if (condition == 0)
    {
        (void)printf("FAIL: %s\n", message);
        exit(1);
    }
}

int main(void)
{
    safety_motion_permit_guard_t guard;
    safety_motion_permit_t       permit = {.base_motion  = 1U,
                                           .generation   = 1UL,
                                           .issued_at_ms = 1000UL,
                                           .valid_for_ms = 40UL};

    SafetyMotionPermitGuard_Init(&guard);
    require_int(SafetyMotionPermitGuard_Evaluate(&guard, &permit, 1040UL) == SAFETY_MOTION_PERMIT_ALLOW,
                "age equal to lease remains valid");
    require_int(SafetyMotionPermitGuard_Evaluate(&guard, &permit, 1041UL) == SAFETY_MOTION_PERMIT_STALE,
                "age above lease is stale");
    require_int(SafetyMotionPermitGuard_Evaluate(&guard, &permit, 1042UL) == SAFETY_MOTION_PERMIT_STALE,
                "stale permit cannot be reused");

    permit.generation   = 2UL;
    permit.issued_at_ms = 1042UL;
    require_int(SafetyMotionPermitGuard_Evaluate(&guard, &permit, 1042UL) == SAFETY_MOTION_PERMIT_GENERATION_CHANGED,
                "permit generation change forces one safe cycle");
    require_int(SafetyMotionPermitGuard_Evaluate(&guard, &permit, 1043UL) == SAFETY_MOTION_PERMIT_ALLOW,
                "new permit is accepted after reset cycle");

    permit.base_motion = 0U;
    permit.generation  = 3UL;
    require_int(SafetyMotionPermitGuard_Evaluate(&guard, &permit, 1043UL) == SAFETY_MOTION_PERMIT_GENERATION_CHANGED,
                "revocation generation is observed before reuse");
    require_int(SafetyMotionPermitGuard_Evaluate(&guard, &permit, 1043UL) == SAFETY_MOTION_PERMIT_REVOKED,
                "revoked permit remains denied");

    permit = (safety_motion_permit_t){.base_motion  = 1U,
                                      .generation   = 4UL,
                                      .issued_at_ms = UINT32_MAX - 10UL,
                                      .valid_for_ms = 40UL};
    require_int(SafetyMotionPermitGuard_Evaluate(&guard, &permit, 9UL) == SAFETY_MOTION_PERMIT_GENERATION_CHANGED,
                "wraparound generation still resets state");
    require_int(SafetyMotionPermitGuard_Evaluate(&guard, &permit, 9UL) == SAFETY_MOTION_PERMIT_ALLOW,
                "unsigned time age is wraparound safe");
    return 0;
}
