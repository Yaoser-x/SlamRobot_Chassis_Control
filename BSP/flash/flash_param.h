#ifndef FLASH_PARAM_H
#define FLASH_PARAM_H

#include <stddef.h>
#include <stdint.h>

#include "param_store.h"
#include "imu_bmi270_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLASH_PARAM_MAGIC 0x464B3037UL
#define FLASH_PARAM_SCHEMA_VERSION 2UL
#define FLASH_PARAM_COMMIT_MARKER 0xC01117EDUL

typedef struct
{
  param_store_t params;
  imu_bmi270_calibration_t imu_calibration;
} flash_param_bundle_t;

#define FLASH_PARAM_IMAGE_SIZE (24U + sizeof(flash_param_bundle_t))
#define FLASH_PARAM_IMAGE_WORD_COUNT (FLASH_PARAM_IMAGE_SIZE / 4U)

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
flash_param_status_t FlashParam_EncodeBundle(const flash_param_bundle_t *bundle,
                                             uint32_t sequence,
                                             uint8_t *image,
                                             size_t image_size);
flash_param_status_t FlashParam_DecodeBundle(const uint8_t *image,
                                             size_t image_size,
                                             flash_param_bundle_t *bundle,
                                             uint32_t *sequence);
flash_param_status_t FlashParam_LoadBundle(flash_param_bundle_t *bundle);
flash_param_status_t FlashParam_SaveBundle(const flash_param_bundle_t *bundle);
flash_param_status_t FlashParam_Load(param_store_t *params);
flash_param_status_t FlashParam_Save(const param_store_t *params);
flash_param_status_t FlashParam_Erase(void);
const char *FlashParam_StatusString(flash_param_status_t status);

#ifdef FLASH_PARAM_HOST_TEST
void FlashParamHost_Reset(void);
void FlashParamHost_SetProgramBudget(int32_t word_budget);
flash_param_status_t FlashParamHost_SeedLegacy(const param_store_t *params);
uint32_t FlashParamHost_GetWatchdogMaintenanceEnterCount(void);
uint32_t FlashParamHost_GetWatchdogMaintenanceExitCount(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* FLASH_PARAM_H */
