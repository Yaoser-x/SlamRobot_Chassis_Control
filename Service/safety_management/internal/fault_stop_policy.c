#include "fault_stop_policy.h"

#include "safety_management_status.h"

#define SAFETY_FAULT_STOP_CAUSE_MASK                                                                                   \
    (SYSTEM_ERROR_LEFT_OVERCURRENT | SYSTEM_ERROR_RIGHT_OVERCURRENT | SYSTEM_ERROR_DRV_FAULT | SYSTEM_ERROR_TIM_BREAK  \
     | SYSTEM_ERROR_ENCODER_FEEDBACK_LOST | SYSTEM_ERROR_BATTERY_CRITICAL)

uint8_t FaultStopPolicy_RequiresFaultStop(uint32_t latched_flags)
{
    return ((latched_flags & SAFETY_FAULT_STOP_CAUSE_MASK) != 0U) ? 1U : 0U;
}

uint32_t FaultStopPolicy_ManualClearMask(uint32_t requested_mask)
{
    return requested_mask & ~SYSTEM_ERROR_BATTERY_CRITICAL;
}
