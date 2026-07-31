#ifndef SAFETY_CAPABILITY_POLICY_H
#define SAFETY_CAPABILITY_POLICY_H

#include "safety_management_config.h"
#include "safety_management_types.h"

void SafetyCapabilityPolicy_Evaluate(const safety_capability_input_t  *input,
                                     const safety_management_config_t *config,
                                     safety_capability_permit_t       *permit);

#endif /* SAFETY_CAPABILITY_POLICY_H */
