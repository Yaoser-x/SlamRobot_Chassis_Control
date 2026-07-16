#include "param_persistence.h"

flash_param_status_t ParamPersistence_Load(flash_param_bundle_t *bundle)
{
    return FlashParam_LoadBundle(bundle);
}

flash_param_status_t ParamPersistence_Save(const flash_param_bundle_t *bundle)
{
    return FlashParam_SaveBundle(bundle);
}
