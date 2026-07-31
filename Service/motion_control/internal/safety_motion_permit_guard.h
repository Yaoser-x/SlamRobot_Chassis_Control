#ifndef SAFETY_MOTION_PERMIT_GUARD_H
#define SAFETY_MOTION_PERMIT_GUARD_H

#include <stdint.h>

#include "safety_management_types.h"

typedef enum
{
    SAFETY_MOTION_PERMIT_INVALID = 0,
    SAFETY_MOTION_PERMIT_ALLOW,
    SAFETY_MOTION_PERMIT_GENERATION_CHANGED,
    SAFETY_MOTION_PERMIT_REVOKED,
    SAFETY_MOTION_PERMIT_STALE
} safety_motion_permit_result_t;

typedef struct
{
    uint32_t expected_generation;
} safety_motion_permit_guard_t;

void                          SafetyMotionPermitGuard_Init(safety_motion_permit_guard_t *guard);
safety_motion_permit_result_t SafetyMotionPermitGuard_Evaluate(safety_motion_permit_guard_t *guard,
                                                               const safety_motion_permit_t *permit,
                                                               uint32_t                      now_ms);

#endif /* SAFETY_MOTION_PERMIT_GUARD_H */
