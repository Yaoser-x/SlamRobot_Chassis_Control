#ifndef PARAM_PERSISTENCE_H
#define PARAM_PERSISTENCE_H

#include "flash_parameter_image.h"

flash_param_status_t ParamPersistence_Load(flash_param_bundle_t *bundle);
flash_param_status_t
ParamPersistence_LoadWithSequence(flash_param_bundle_t *bundle, uint32_t *sequence, uint8_t *sequence_valid);
flash_param_status_t ParamPersistence_Save(const flash_param_bundle_t *bundle);

#endif
