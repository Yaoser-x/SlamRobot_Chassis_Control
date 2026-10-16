#include "parameter_persistence_backend.h"

flash_param_status_t ParamPersistence_Load(flash_param_bundle_t *bundle)
{
    return FlashParam_LoadBundle(bundle);
}

flash_param_status_t
ParamPersistence_LoadWithSequence(flash_param_bundle_t *bundle, uint32_t *sequence, uint8_t *sequence_valid)
{
    return FlashParam_LoadBundleWithSequence(bundle, sequence, sequence_valid);
}

flash_param_status_t ParamPersistence_Save(const flash_param_bundle_t *bundle)
{
    return FlashParam_SaveBundle(bundle);
}
