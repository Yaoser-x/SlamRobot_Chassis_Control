#include "flash_param.h"

#include <string.h>

#ifndef FLASH_PARAM_HOST_TEST
#include "iwdg.h"
#include "stm32f4xx_hal.h"
#endif

#define FLASH_PARAM_STORAGE_ADDR 0x08060000UL
#define FLASH_PARAM_STORAGE_SECTOR FLASH_SECTOR_7

typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t data_size;
  uint32_t crc32;
  param_store_t data;
} flash_param_image_t;

static uint32_t FlashParam_Crc32(const uint8_t *data, size_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;

  for (size_t i = 0U; i < len; ++i)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      uint32_t mask = (uint32_t)(0UL - (crc & 1UL));
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

static uint8_t FlashParam_IsBlank(const uint8_t *image, size_t image_size)
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

flash_param_status_t FlashParam_Encode(const param_store_t *params, uint8_t *image, size_t image_size)
{
  flash_param_image_t encoded;

  if (params == 0 || image == 0 || image_size < sizeof(encoded))
  {
    return FLASH_PARAM_STATUS_INVALID;
  }
  if (ParamStore_Validate(params) == 0U)
  {
    return FLASH_PARAM_STATUS_INVALID;
  }

  encoded = (flash_param_image_t){0};
  encoded.magic = FLASH_PARAM_MAGIC;
  encoded.version = PARAM_STORE_VERSION;
  encoded.data_size = sizeof(param_store_t);
  encoded.data = *params;
  encoded.crc32 = FlashParam_Crc32((const uint8_t *)&encoded.data,
                                   sizeof(encoded.data));
  memcpy(image, &encoded, sizeof(encoded));
  return FLASH_PARAM_STATUS_OK;
}

flash_param_status_t FlashParam_Decode(const uint8_t *image, size_t image_size, param_store_t *params)
{
  const flash_param_image_t *encoded;
  uint32_t crc;

  if (image == 0 || params == 0 || image_size < sizeof(flash_param_image_t))
  {
    return FLASH_PARAM_STATUS_INVALID;
  }
  if (FlashParam_IsBlank(image, sizeof(flash_param_image_t)) != 0U)
  {
    return FLASH_PARAM_STATUS_EMPTY;
  }

  encoded = (const flash_param_image_t *)image;
  if (encoded->magic != FLASH_PARAM_MAGIC ||
      encoded->version != PARAM_STORE_VERSION ||
      encoded->data_size != sizeof(param_store_t))
  {
    return FLASH_PARAM_STATUS_UNSUPPORTED;
  }

  crc = FlashParam_Crc32((const uint8_t *)&encoded->data,
                         sizeof(encoded->data));
  if (crc != encoded->crc32)
  {
    return FLASH_PARAM_STATUS_CRC_ERROR;
  }
  if (ParamStore_Validate(&encoded->data) == 0U)
  {
    return FLASH_PARAM_STATUS_INVALID;
  }

  *params = encoded->data;
  return FLASH_PARAM_STATUS_OK;
}

const char *FlashParam_StatusString(flash_param_status_t status)
{
  switch (status)
  {
    case FLASH_PARAM_STATUS_OK:          return "ok";
    case FLASH_PARAM_STATUS_EMPTY:       return "empty";
    case FLASH_PARAM_STATUS_CRC_ERROR:   return "crc";
    case FLASH_PARAM_STATUS_UNSUPPORTED: return "unsupported";
    case FLASH_PARAM_STATUS_INVALID:     return "invalid";
    case FLASH_PARAM_STATUS_WRITE_ERROR: return "write";
    default:                             return "unknown";
  }
}

#ifdef FLASH_PARAM_HOST_TEST
flash_param_status_t FlashParam_Load(param_store_t *params)
{
  (void)params;
  return FLASH_PARAM_STATUS_UNSUPPORTED;
}

flash_param_status_t FlashParam_Save(const param_store_t *params)
{
  (void)params;
  return FLASH_PARAM_STATUS_UNSUPPORTED;
}

flash_param_status_t FlashParam_Erase(void)
{
  return FLASH_PARAM_STATUS_UNSUPPORTED;
}
#else
flash_param_status_t FlashParam_Load(param_store_t *params)
{
  return FlashParam_Decode((const uint8_t *)FLASH_PARAM_STORAGE_ADDR,
                           sizeof(flash_param_image_t),
                           params);
}

flash_param_status_t FlashParam_Erase(void)
{
  HAL_StatusTypeDef hal_status;
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t sector_error = 0UL;

  HAL_IWDG_Refresh(&hiwdg);
  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return FLASH_PARAM_STATUS_WRITE_ERROR;
  }
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = FLASH_PARAM_STORAGE_SECTOR;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  hal_status = HAL_FLASHEx_Erase(&erase, &sector_error);
  (void)HAL_FLASH_Lock();
  HAL_IWDG_Refresh(&hiwdg);

  return (hal_status == HAL_OK && sector_error == 0xFFFFFFFFUL) ?
         FLASH_PARAM_STATUS_OK : FLASH_PARAM_STATUS_WRITE_ERROR;
}

flash_param_status_t FlashParam_Save(const param_store_t *params)
{
  uint8_t image[sizeof(flash_param_image_t)];
  flash_param_status_t status;
  uint32_t addr = FLASH_PARAM_STORAGE_ADDR;

  status = FlashParam_Encode(params, image, sizeof(image));
  if (status != FLASH_PARAM_STATUS_OK)
  {
    return status;
  }
  status = FlashParam_Erase();
  if (status != FLASH_PARAM_STATUS_OK)
  {
    return status;
  }

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return FLASH_PARAM_STATUS_WRITE_ERROR;
  }
  for (size_t offset = 0U; offset < sizeof(image); offset += 4U)
  {
    uint32_t word = 0xFFFFFFFFUL;
    size_t remaining = sizeof(image) - offset;
    size_t copy_len = (remaining >= 4U) ? 4U : remaining;
    memcpy(&word, &image[offset], copy_len);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, word) != HAL_OK)
    {
      (void)HAL_FLASH_Lock();
      return FLASH_PARAM_STATUS_WRITE_ERROR;
    }
    addr += 4UL;
  }
  (void)HAL_FLASH_Lock();
  return FLASH_PARAM_STATUS_OK;
}
#endif
