#ifndef FLASH_PARAMETER_MIGRATION_H
#define FLASH_PARAMETER_MIGRATION_H

#include "flash_param_schema.h"

void FlashParamMigrate_FromV1(const param_model_v1_t *old, param_model_t *params);
void FlashParamMigrate_FromV2(const flash_param_bundle_v2_t *old, flash_param_bundle_t *bundle);
void FlashParamMigrate_FromV3(const flash_param_bundle_v3_t *old, flash_param_bundle_t *bundle);

#endif /* FLASH_PARAM_MIGRATION_H */
