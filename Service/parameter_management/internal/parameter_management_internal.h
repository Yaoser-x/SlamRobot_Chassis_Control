#ifndef PARAMETER_MANAGEMENT_INTERNAL_H
#define PARAMETER_MANAGEMENT_INTERNAL_H

#include "flash_param.h"

/** Internal persistence bridge; not available to other capabilities. */
void ParameterManagementInternal_ApplyLoaded(const flash_param_bundle_t *bundle);
void ParameterManagementInternal_BuildBundle(flash_param_bundle_t *bundle);

#endif
