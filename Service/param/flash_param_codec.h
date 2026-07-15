#ifndef FLASH_PARAM_CODEC_H
#define FLASH_PARAM_CODEC_H

#include "flash_param_schema.h"

uint32_t FlashParamCodec_Crc32(const uint8_t *data, size_t len);
uint8_t  FlashParamCodec_BundleValid(const flash_param_bundle_t *bundle);
flash_param_status_t
FlashParamCodec_EncodeBundle(const flash_param_bundle_t *bundle, uint32_t sequence, uint8_t *image, size_t image_size);
flash_param_status_t
FlashParamCodec_DecodeBundle(const uint8_t *image, size_t image_size, flash_param_bundle_t *bundle, uint32_t *sequence);
flash_param_status_t
FlashParamCodec_DecodeLegacy(const uint8_t *image, size_t image_size, flash_param_bundle_t *bundle);

#endif /* FLASH_PARAM_CODEC_H */
