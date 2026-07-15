#include "flash_param_codec.h"

#include "flash_param_migration.h"

#include <stddef.h>
#include <string.h>

uint32_t FlashParamCodec_Crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;

    for (size_t i = 0U; i < len; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            uint32_t mask = (uint32_t)(0UL - (crc & 1UL));
            crc           = (crc >> 1) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}

static uint8_t FlashParamCodec_IsBlank(const uint8_t *image, size_t image_size)
{
    for (size_t i = 0U; i < image_size; ++i)
    {
        if (image[i] != 0xFFU)
        {
            return 0U;
        }
    }
    return 1U;
}

uint8_t FlashParamCodec_BundleValid(const flash_param_bundle_t *bundle)
{
    if (bundle == 0 || ParamService_Validate(&bundle->params) == 0U)
    {
        return 0U;
    }
    return ImuBmi270Calibration_Validate(&bundle->imu_calibration);
}

flash_param_status_t
FlashParamCodec_EncodeBundle(const flash_param_bundle_t *bundle, uint32_t sequence, uint8_t *image, size_t image_size)
{
    flash_param_image_t encoded;

    if (bundle == 0 || image == 0 || image_size < sizeof(encoded) || FlashParamCodec_BundleValid(bundle) == 0U)
    {
        return FLASH_PARAM_STATUS_INVALID;
    }
    encoded                                    = (flash_param_image_t){0};
    encoded.magic                              = FLASH_PARAM_MAGIC;
    encoded.schema_version                     = FLASH_PARAM_SCHEMA_VERSION;
    encoded.sequence                           = sequence;
    encoded.payload_size                       = sizeof(encoded.payload);
    encoded.payload                            = *bundle;
    encoded.payload.params.imu_gyro_bias_valid = 0U;
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        encoded.payload.params.imu_gyro_bias_dps[i] = 0.0f;
    }
    encoded.crc32         = FlashParamCodec_Crc32((const uint8_t *)&encoded, offsetof(flash_param_image_t, crc32));
    encoded.commit_marker = FLASH_PARAM_COMMIT_MARKER;
    memcpy(image, &encoded, sizeof(encoded));
    return FLASH_PARAM_STATUS_OK;
}

flash_param_status_t
FlashParamCodec_DecodeBundle(const uint8_t *image, size_t image_size, flash_param_bundle_t *bundle, uint32_t *sequence)
{
    const flash_param_image_t *encoded;
    uint32_t                   crc;

    if (image == 0 || bundle == 0 || image_size < (sizeof(uint32_t) * 2U))
    {
        return FLASH_PARAM_STATUS_INVALID;
    }
    if (FlashParamCodec_IsBlank(image, image_size) != 0U)
    {
        return FLASH_PARAM_STATUS_EMPTY;
    }
    encoded = (const flash_param_image_t *)image;
    if (encoded->magic == FLASH_PARAM_MAGIC && encoded->schema_version == 2UL)
    {
        const flash_param_image_v2_t *old = (const flash_param_image_v2_t *)image;
        if (image_size < sizeof(*old) || old->payload_size != sizeof(old->payload)
            || old->commit_marker != FLASH_PARAM_COMMIT_MARKER)
        {
            return FLASH_PARAM_STATUS_UNSUPPORTED;
        }
        crc = FlashParamCodec_Crc32((const uint8_t *)old, offsetof(flash_param_image_v2_t, crc32));
        if (crc != old->crc32)
        {
            return FLASH_PARAM_STATUS_CRC_ERROR;
        }
        FlashParamMigrate_FromV2(&old->payload, bundle);
        if (ParamService_Validate(&bundle->params) == 0U)
        {
            return FLASH_PARAM_STATUS_INVALID;
        }
        if (sequence != 0)
        {
            *sequence = old->sequence;
        }
        return FLASH_PARAM_STATUS_OK;
    }
    if (encoded->magic == FLASH_PARAM_MAGIC && encoded->schema_version == 3UL)
    {
        const flash_param_image_v3_t *old = (const flash_param_image_v3_t *)image;
        if (image_size < sizeof(*old) || old->payload_size != sizeof(old->payload)
            || old->commit_marker != FLASH_PARAM_COMMIT_MARKER)
        {
            return FLASH_PARAM_STATUS_UNSUPPORTED;
        }
        crc = FlashParamCodec_Crc32((const uint8_t *)old, offsetof(flash_param_image_v3_t, crc32));
        if (crc != old->crc32)
        {
            return FLASH_PARAM_STATUS_CRC_ERROR;
        }
        FlashParamMigrate_FromV3(&old->payload, bundle);
        if (ParamService_Validate(&bundle->params) == 0U)
        {
            return FLASH_PARAM_STATUS_INVALID;
        }
        if (sequence != 0)
        {
            *sequence = old->sequence;
        }
        return FLASH_PARAM_STATUS_OK;
    }
    if (image_size < sizeof(*encoded))
    {
        return FLASH_PARAM_STATUS_INVALID;
    }
    if (encoded->magic != FLASH_PARAM_MAGIC || encoded->schema_version != FLASH_PARAM_SCHEMA_VERSION
        || encoded->payload_size != sizeof(encoded->payload))
    {
        return FLASH_PARAM_STATUS_UNSUPPORTED;
    }
    if (encoded->commit_marker != FLASH_PARAM_COMMIT_MARKER)
    {
        return FLASH_PARAM_STATUS_CRC_ERROR;
    }
    crc = FlashParamCodec_Crc32((const uint8_t *)encoded, offsetof(flash_param_image_t, crc32));
    if (crc != encoded->crc32)
    {
        return FLASH_PARAM_STATUS_CRC_ERROR;
    }
    if (FlashParamCodec_BundleValid(&encoded->payload) == 0U)
    {
        return FLASH_PARAM_STATUS_INVALID;
    }
    *bundle = encoded->payload;
    if (sequence != 0)
    {
        *sequence = encoded->sequence;
    }
    return FLASH_PARAM_STATUS_OK;
}

flash_param_status_t FlashParamCodec_DecodeLegacy(const uint8_t *image, size_t image_size, flash_param_bundle_t *bundle)
{
    const flash_param_legacy_image_t *legacy;
    uint32_t                          crc;

    if (image == 0 || bundle == 0 || image_size < sizeof(flash_param_legacy_image_t))
    {
        return FLASH_PARAM_STATUS_INVALID;
    }
    if (FlashParamCodec_IsBlank(image, sizeof(flash_param_legacy_image_t)) != 0U)
    {
        return FLASH_PARAM_STATUS_EMPTY;
    }
    legacy = (const flash_param_legacy_image_t *)image;
    if (legacy->magic != FLASH_PARAM_MAGIC || legacy->version != 1UL || legacy->data_size != sizeof(param_model_v1_t))
    {
        return FLASH_PARAM_STATUS_UNSUPPORTED;
    }
    crc = FlashParamCodec_Crc32((const uint8_t *)&legacy->data, sizeof(legacy->data));
    if (crc != legacy->crc32)
    {
        return FLASH_PARAM_STATUS_CRC_ERROR;
    }
    FlashParamMigrate_FromV1(&legacy->data, &bundle->params);
    ImuBmi270Calibration_Default(&bundle->imu_calibration);
    return FLASH_PARAM_STATUS_OK;
}
