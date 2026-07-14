#ifndef PARAM_PERSISTENCE_H
#define PARAM_PERSISTENCE_H

#include "flash_param.h"

flash_param_status_t ParamPersistence_Load(flash_param_bundle_t *bundle);
flash_param_status_t ParamPersistence_Save(const flash_param_bundle_t *bundle);

#endif
