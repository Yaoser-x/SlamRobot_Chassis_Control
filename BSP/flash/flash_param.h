#ifndef FLASH_PARAM_H
#define FLASH_PARAM_H

#include <stddef.h>
#include <stdint.h>

#include "param_store.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLASH_PARAM_MAGIC      0x464B3037UL
#define FLASH_PARAM_IMAGE_SIZE (16U + sizeof(param_store_t))

typedef enum
{
  FLASH_PARAM_STATUS_OK = 0,
  FLASH_PARAM_STATUS_EMPTY,
  FLASH_PARAM_STATUS_CRC_ERROR,
  FLASH_PARAM_STATUS_UNSUPPORTED,
  FLASH_PARAM_STATUS_INVALID,
  FLASH_PARAM_STATUS_WRITE_ERROR
} flash_param_status_t;

flash_param_status_t FlashParam_Encode(const param_store_t *params, uint8_t *image, size_t image_size);
flash_param_status_t FlashParam_Decode(const uint8_t *image, size_t image_size, param_store_t *params);
flash_param_status_t FlashParam_Load(param_store_t *params);
flash_param_status_t FlashParam_Save(const param_store_t *params);
flash_param_status_t FlashParam_Erase(void);
const char *FlashParam_StatusString(flash_param_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_PARAM_H */
