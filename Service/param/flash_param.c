#include "flash_param.h"
#include "flash_param_codec.h"
#include "flash_param_schema.h"
#include "flash_param_slots.h"
#include "param_service.h"
#include "flash_storage.h"

#include <stddef.h>
#include <string.h>

#define FLASH_PARAM_SLOT_COUNT 2U

flash_param_status_t
FlashParam_EncodeBundle(const flash_param_bundle_t *bundle, uint32_t sequence, uint8_t *image, size_t image_size)
{
    return FlashParamCodec_EncodeBundle(bundle, sequence, image, image_size);
}

flash_param_status_t
FlashParam_DecodeBundle(const uint8_t *image, size_t image_size, flash_param_bundle_t *bundle, uint32_t *sequence)
{
    return FlashParamCodec_DecodeBundle(image, image_size, bundle, sequence);
}

flash_param_status_t FlashParam_Encode(const param_model_t *params, uint8_t *image, size_t image_size)
{
    flash_param_bundle_t bundle;

    if (params == 0)
    {
        return FLASH_PARAM_STATUS_INVALID;
    }
    bundle.params = *params;
    ImuBmi270Calibration_Default(&bundle.imu_calibration);
    return FlashParam_EncodeBundle(&bundle, 0UL, image, image_size);
}

flash_param_status_t FlashParam_Decode(const uint8_t *image, size_t image_size, param_model_t *params)
{
    flash_param_bundle_t bundle;
    flash_param_status_t status;

    if (params == 0)
    {
        return FLASH_PARAM_STATUS_INVALID;
    }
    status = FlashParam_DecodeBundle(image, image_size, &bundle, 0);
    if (status == FLASH_PARAM_STATUS_OK)
    {
        *params = bundle.params;
    }
    return status;
}

flash_param_status_t FlashParam_LoadBundle(flash_param_bundle_t *bundle)
{
    return FlashParamSlots_LoadBundle(bundle);
}

flash_param_status_t FlashParam_SaveBundle(const flash_param_bundle_t *bundle)
{
    return FlashParamSlots_SaveBundle(bundle);
}

flash_param_status_t FlashParam_Load(param_model_t *params)
{
    flash_param_bundle_t bundle;
    flash_param_status_t status;

    if (params == 0)
    {
        return FLASH_PARAM_STATUS_INVALID;
    }
    status = FlashParam_LoadBundle(&bundle);
    if (status == FLASH_PARAM_STATUS_OK)
    {
        *params = bundle.params;
    }
    return status;
}

flash_param_status_t FlashParam_Save(const param_model_t *params)
{
    flash_param_bundle_t bundle;

    if (params == 0)
    {
        return FLASH_PARAM_STATUS_INVALID;
    }
    bundle.params = *params;
    ImuBmi270Calibration_Default(&bundle.imu_calibration);
    return FlashParam_SaveBundle(&bundle);
}

flash_param_status_t FlashParam_Erase(void)
{
    return FlashParamSlots_Erase();
}

const char *FlashParam_StatusString(flash_param_status_t status)
{
    switch (status)
    {
        case FLASH_PARAM_STATUS_OK:
            return "ok";
        case FLASH_PARAM_STATUS_EMPTY:
            return "empty";
        case FLASH_PARAM_STATUS_CRC_ERROR:
            return "crc";
        case FLASH_PARAM_STATUS_UNSUPPORTED:
            return "unsupported";
        case FLASH_PARAM_STATUS_INVALID:
            return "invalid";
        case FLASH_PARAM_STATUS_WRITE_ERROR:
            return "write";
        default:
            return "unknown";
    }
}
