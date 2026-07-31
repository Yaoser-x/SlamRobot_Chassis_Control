#include "safety_motion_permit_guard.h"

void SafetyMotionPermitGuard_Init(safety_motion_permit_guard_t *guard)
{
    if (guard != 0)
    {
        guard->expected_generation = 0UL;
    }
}

safety_motion_permit_result_t SafetyMotionPermitGuard_Evaluate(safety_motion_permit_guard_t *guard,
                                                               const safety_motion_permit_t *permit,
                                                               uint32_t                      now_ms)
{
    if (guard == 0 || permit == 0 || permit->generation == 0UL)
    {
        return SAFETY_MOTION_PERMIT_INVALID;
    }
    if (guard->expected_generation == 0UL)
    {
        guard->expected_generation = permit->generation;
    }
    else if (guard->expected_generation != permit->generation)
    {
        guard->expected_generation = permit->generation;
        return SAFETY_MOTION_PERMIT_GENERATION_CHANGED;
    }
    if (permit->base_motion == 0U && permit->maintenance_motion == 0U)
    {
        return SAFETY_MOTION_PERMIT_REVOKED;
    }
    if (permit->valid_for_ms == 0UL || (uint32_t)(now_ms - permit->issued_at_ms) > permit->valid_for_ms)
    {
        return SAFETY_MOTION_PERMIT_STALE;
    }
    return SAFETY_MOTION_PERMIT_ALLOW;
}
