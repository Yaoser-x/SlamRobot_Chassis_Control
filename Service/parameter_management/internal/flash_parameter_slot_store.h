#ifndef FLASH_PARAM_SLOTS_H
#define FLASH_PARAM_SLOTS_H

#include "flash_parameter_image.h"

flash_param_status_t FlashParamSlots_LoadBundle(flash_param_bundle_t *bundle);
flash_param_status_t FlashParamSlots_SaveBundle(const flash_param_bundle_t *bundle);
flash_param_status_t FlashParamSlots_Erase(void);

#endif /* FLASH_PARAM_SLOTS_H */
