#ifndef SAFETY_FAULT_POLICY_H
#define SAFETY_FAULT_POLICY_H

#include <stdint.h>

uint8_t  SafetyFaultPolicy_RequiresFaultStop(uint32_t latched_flags);
uint32_t SafetyFaultPolicy_ManualClearMask(uint32_t requested_mask);

#endif /* SAFETY_FAULT_POLICY_H */
