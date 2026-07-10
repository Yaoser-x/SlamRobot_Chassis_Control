#include "flash_param.h"

#include <stddef.h>
#include <string.h>

#ifndef FLASH_PARAM_HOST_TEST
#include "iwdg.h"
#include "stm32f4xx_hal.h"
#endif

#define FLASH_PARAM_STORAGE_ADDR_A 0x08040000UL
#define FLASH_PARAM_STORAGE_ADDR_B 0x08060000UL
#define FLASH_PARAM_STORAGE_SECTOR_A FLASH_SECTOR_6
#define FLASH_PARAM_STORAGE_SECTOR_B FLASH_SECTOR_7
#define FLASH_PARAM_SLOT_COUNT 2U

typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t data_size;
  uint32_t crc32;
  param_store_t data;
} flash_param_legacy_image_t;

typedef struct
{
  uint32_t magic;
  uint32_t schema_version;
  uint32_t sequence;
  uint32_t payload_size;
  flash_param_bundle_t payload;
  uint32_t crc32;
  uint32_t commit_marker;
} flash_param_image_t;

_Static_assert(sizeof(flash_param_image_t) == FLASH_PARAM_IMAGE_SIZE,
               "flash parameter image size mismatch");
_Static_assert((FLASH_PARAM_IMAGE_SIZE % 4U) == 0U,
               "flash parameter image must be word aligned");

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

static uint8_t FlashParam_BundleValid(const flash_param_bundle_t *bundle)
{
  if (bundle == 0 || ParamStore_Validate(&bundle->params) == 0U)
  {
    return 0U;
  }
  return ImuBmi270Calibration_Validate(&bundle->imu_calibration);
}

flash_param_status_t FlashParam_EncodeBundle(const flash_param_bundle_t *bundle,
                                             uint32_t sequence,
                                             uint8_t *image,
                                             size_t image_size)
{
  flash_param_image_t encoded;

  if (bundle == 0 || image == 0 || image_size < sizeof(encoded))
  {
    return FLASH_PARAM_STATUS_INVALID;
  }
  if (FlashParam_BundleValid(bundle) == 0U)
  {
    return FLASH_PARAM_STATUS_INVALID;
  }
  encoded = (flash_param_image_t){0};
  encoded.magic = FLASH_PARAM_MAGIC;
  encoded.schema_version = FLASH_PARAM_SCHEMA_VERSION;
  encoded.sequence = sequence;
  encoded.payload_size = sizeof(encoded.payload);
  encoded.payload = *bundle;
  encoded.crc32 = FlashParam_Crc32((const uint8_t *)&encoded,
                                   offsetof(flash_param_image_t, crc32));
  encoded.commit_marker = FLASH_PARAM_COMMIT_MARKER;
  memcpy(image, &encoded, sizeof(encoded));
  return FLASH_PARAM_STATUS_OK;
}

flash_param_status_t FlashParam_DecodeBundle(const uint8_t *image,
                                             size_t image_size,
                                             flash_param_bundle_t *bundle,
                                             uint32_t *sequence)
{
  const flash_param_image_t *encoded;
  uint32_t crc;

  if (image == 0 || bundle == 0 || image_size < sizeof(flash_param_image_t))
  {
    return FLASH_PARAM_STATUS_INVALID;
  }
  if (FlashParam_IsBlank(image, sizeof(flash_param_image_t)) != 0U)
  {
    return FLASH_PARAM_STATUS_EMPTY;
  }
  encoded = (const flash_param_image_t *)image;
  if (encoded->magic != FLASH_PARAM_MAGIC ||
      encoded->schema_version != FLASH_PARAM_SCHEMA_VERSION ||
      encoded->payload_size != sizeof(encoded->payload))
  {
    return FLASH_PARAM_STATUS_UNSUPPORTED;
  }
  if (encoded->commit_marker != FLASH_PARAM_COMMIT_MARKER)
  {
    return FLASH_PARAM_STATUS_CRC_ERROR;
  }
  crc = FlashParam_Crc32((const uint8_t *)encoded,
                         offsetof(flash_param_image_t, crc32));
  if (crc != encoded->crc32)
  {
    return FLASH_PARAM_STATUS_CRC_ERROR;
  }
  if (FlashParam_BundleValid(&encoded->payload) == 0U)
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

flash_param_status_t FlashParam_Encode(const param_store_t *params,
                                       uint8_t *image,
                                       size_t image_size)
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

flash_param_status_t FlashParam_Decode(const uint8_t *image,
                                       size_t image_size,
                                       param_store_t *params)
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

static flash_param_status_t FlashParam_DecodeLegacy(const uint8_t *image,
                                                    size_t image_size,
                                                    flash_param_bundle_t *bundle)
{
  const flash_param_legacy_image_t *legacy;
  uint32_t crc;

  if (image == 0 || bundle == 0 || image_size < sizeof(flash_param_legacy_image_t))
  {
    return FLASH_PARAM_STATUS_INVALID;
  }
  if (FlashParam_IsBlank(image, sizeof(flash_param_legacy_image_t)) != 0U)
  {
    return FLASH_PARAM_STATUS_EMPTY;
  }
  legacy = (const flash_param_legacy_image_t *)image;
  if (legacy->magic != FLASH_PARAM_MAGIC ||
      legacy->version != PARAM_STORE_VERSION ||
      legacy->data_size != sizeof(param_store_t))
  {
    return FLASH_PARAM_STATUS_UNSUPPORTED;
  }
  crc = FlashParam_Crc32((const uint8_t *)&legacy->data, sizeof(legacy->data));
  if (crc != legacy->crc32)
  {
    return FLASH_PARAM_STATUS_CRC_ERROR;
  }
  if (ParamStore_Validate(&legacy->data) == 0U)
  {
    return FLASH_PARAM_STATUS_INVALID;
  }
  bundle->params = legacy->data;
  ImuBmi270Calibration_Default(&bundle->imu_calibration);
  return FLASH_PARAM_STATUS_OK;
}

#ifdef FLASH_PARAM_HOST_TEST
static uint8_t flash_param_host_slots[FLASH_PARAM_SLOT_COUNT][FLASH_PARAM_IMAGE_SIZE];
static int32_t flash_param_host_program_budget = -1;
static uint32_t flash_param_host_watchdog_enter_count;
static uint32_t flash_param_host_watchdog_exit_count;

void FlashParamHost_Reset(void)
{
  memset(flash_param_host_slots, 0xFF, sizeof(flash_param_host_slots));
  flash_param_host_program_budget = -1;
  flash_param_host_watchdog_enter_count = 0UL;
  flash_param_host_watchdog_exit_count = 0UL;
}

void FlashParamHost_SetProgramBudget(int32_t word_budget)
{
  flash_param_host_program_budget = word_budget;
}

uint32_t FlashParamHost_GetWatchdogMaintenanceEnterCount(void)
{
  return flash_param_host_watchdog_enter_count;
}

uint32_t FlashParamHost_GetWatchdogMaintenanceExitCount(void)
{
  return flash_param_host_watchdog_exit_count;
}

static flash_param_status_t FlashParam_WatchdogEnterMaintenance(void)
{
  flash_param_host_watchdog_enter_count++;
  return FLASH_PARAM_STATUS_OK;
}

static flash_param_status_t FlashParam_WatchdogExitMaintenance(void)
{
  flash_param_host_watchdog_exit_count++;
  return FLASH_PARAM_STATUS_OK;
}

static const uint8_t *FlashParam_SlotData(uint8_t slot)
{
  return flash_param_host_slots[slot];
}

static flash_param_status_t FlashParam_StorageUnlock(void)
{
  return FLASH_PARAM_STATUS_OK;
}

static void FlashParam_StorageLock(void)
{
}

static flash_param_status_t FlashParam_EraseSlot(uint8_t slot)
{
  memset(flash_param_host_slots[slot], 0xFF, FLASH_PARAM_IMAGE_SIZE);
  return FLASH_PARAM_STATUS_OK;
}

static flash_param_status_t FlashParam_ProgramWord(uint8_t slot,
                                                  size_t offset,
                                                  uint32_t word)
{
  uint32_t current;

  if (flash_param_host_program_budget == 0)
  {
    return FLASH_PARAM_STATUS_WRITE_ERROR;
  }
  if (flash_param_host_program_budget > 0)
  {
    flash_param_host_program_budget--;
  }
  memcpy(&current, &flash_param_host_slots[slot][offset], sizeof(current));
  if ((current & word) != word)
  {
    return FLASH_PARAM_STATUS_WRITE_ERROR;
  }
  memcpy(&flash_param_host_slots[slot][offset], &word, sizeof(word));
  return FLASH_PARAM_STATUS_OK;
}

flash_param_status_t FlashParamHost_SeedLegacy(const param_store_t *params)
{
  flash_param_legacy_image_t legacy;

  if (ParamStore_Validate(params) == 0U)
  {
    return FLASH_PARAM_STATUS_INVALID;
  }
  legacy = (flash_param_legacy_image_t){0};
  legacy.magic = FLASH_PARAM_MAGIC;
  legacy.version = PARAM_STORE_VERSION;
  legacy.data_size = sizeof(legacy.data);
  legacy.data = *params;
  legacy.crc32 = FlashParam_Crc32((const uint8_t *)&legacy.data, sizeof(legacy.data));
  memset(flash_param_host_slots[1], 0xFF, FLASH_PARAM_IMAGE_SIZE);
  memcpy(flash_param_host_slots[1], &legacy, sizeof(legacy));
  return FLASH_PARAM_STATUS_OK;
}
#else
static uint32_t flash_param_watchdog_saved_prescaler;
static uint32_t flash_param_watchdog_saved_reload;

static flash_param_status_t FlashParam_WatchdogEnterMaintenance(void)
{
  flash_param_watchdog_saved_prescaler = hiwdg.Init.Prescaler;
  flash_param_watchdog_saved_reload = hiwdg.Init.Reload;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095U;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    hiwdg.Init.Prescaler = flash_param_watchdog_saved_prescaler;
    hiwdg.Init.Reload = flash_param_watchdog_saved_reload;
    return FLASH_PARAM_STATUS_WRITE_ERROR;
  }
  return FLASH_PARAM_STATUS_OK;
}

static flash_param_status_t FlashParam_WatchdogExitMaintenance(void)
{
  hiwdg.Init.Prescaler = flash_param_watchdog_saved_prescaler;
  hiwdg.Init.Reload = flash_param_watchdog_saved_reload;
  return (HAL_IWDG_Init(&hiwdg) == HAL_OK) ? FLASH_PARAM_STATUS_OK :
                                            FLASH_PARAM_STATUS_WRITE_ERROR;
}

static const uint8_t *FlashParam_SlotData(uint8_t slot)
{
  uint32_t address = (slot == 0U) ? FLASH_PARAM_STORAGE_ADDR_A : FLASH_PARAM_STORAGE_ADDR_B;
  return (const uint8_t *)address;
}

static flash_param_status_t FlashParam_StorageUnlock(void)
{
  HAL_IWDG_Refresh(&hiwdg);
  return (HAL_FLASH_Unlock() == HAL_OK) ? FLASH_PARAM_STATUS_OK :
                                         FLASH_PARAM_STATUS_WRITE_ERROR;
}

static void FlashParam_StorageLock(void)
{
  (void)HAL_FLASH_Lock();
  HAL_IWDG_Refresh(&hiwdg);
}

static flash_param_status_t FlashParam_EraseSlot(uint8_t slot)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t sector_error = 0UL;
  HAL_StatusTypeDef hal_status;

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = (slot == 0U) ? FLASH_PARAM_STORAGE_SECTOR_A :
                               FLASH_PARAM_STORAGE_SECTOR_B;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  hal_status = HAL_FLASHEx_Erase(&erase, &sector_error);
  return (hal_status == HAL_OK && sector_error == 0xFFFFFFFFUL) ?
         FLASH_PARAM_STATUS_OK : FLASH_PARAM_STATUS_WRITE_ERROR;
}

static flash_param_status_t FlashParam_ProgramWord(uint8_t slot,
                                                  size_t offset,
                                                  uint32_t word)
{
  uint32_t address = ((slot == 0U) ? FLASH_PARAM_STORAGE_ADDR_A :
                                        FLASH_PARAM_STORAGE_ADDR_B) +
                     (uint32_t)offset;
  return (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, word) == HAL_OK) ?
         FLASH_PARAM_STATUS_OK : FLASH_PARAM_STATUS_WRITE_ERROR;
}
#endif

static uint8_t FlashParam_SequenceNewer(uint32_t lhs, uint32_t rhs)
{
  return (((int32_t)(lhs - rhs)) > 0) ? 1U : 0U;
}

static int8_t FlashParam_SelectActiveSlot(uint32_t *sequence)
{
  flash_param_bundle_t bundle;
  uint32_t slot_sequence[FLASH_PARAM_SLOT_COUNT] = {0UL, 0UL};
  uint8_t valid[FLASH_PARAM_SLOT_COUNT] = {0U, 0U};

  for (uint8_t slot = 0U; slot < FLASH_PARAM_SLOT_COUNT; ++slot)
  {
    if (FlashParam_DecodeBundle(FlashParam_SlotData(slot),
                                FLASH_PARAM_IMAGE_SIZE,
                                &bundle,
                                &slot_sequence[slot]) == FLASH_PARAM_STATUS_OK)
    {
      valid[slot] = 1U;
    }
  }
  if (valid[0] != 0U && valid[1] != 0U)
  {
    uint8_t selected = (FlashParam_SequenceNewer(slot_sequence[1], slot_sequence[0]) != 0U) ? 1U : 0U;
    if (sequence != 0)
    {
      *sequence = slot_sequence[selected];
    }
    return (int8_t)selected;
  }
  if (valid[0] != 0U || valid[1] != 0U)
  {
    uint8_t selected = (valid[1] != 0U) ? 1U : 0U;
    if (sequence != 0)
    {
      *sequence = slot_sequence[selected];
    }
    return (int8_t)selected;
  }
  if (sequence != 0)
  {
    *sequence = 0UL;
  }
  return -1;
}

flash_param_status_t FlashParam_LoadBundle(flash_param_bundle_t *bundle)
{
  int8_t active;
  flash_param_status_t legacy_status;

  if (bundle == 0)
  {
    return FLASH_PARAM_STATUS_INVALID;
  }
  active = FlashParam_SelectActiveSlot(0);
  if (active >= 0)
  {
    return FlashParam_DecodeBundle(FlashParam_SlotData((uint8_t)active),
                                   FLASH_PARAM_IMAGE_SIZE,
                                   bundle,
                                   0);
  }
  legacy_status = FlashParam_DecodeLegacy(FlashParam_SlotData(1U),
                                         FLASH_PARAM_IMAGE_SIZE,
                                         bundle);
  return legacy_status;
}

flash_param_status_t FlashParam_SaveBundle(const flash_param_bundle_t *bundle)
{
  uint8_t image[FLASH_PARAM_IMAGE_SIZE];
  flash_param_bundle_t verify_bundle;
  uint32_t active_sequence = 0UL;
  uint32_t verify_sequence = 0UL;
  int8_t active;
  uint8_t target;
  flash_param_status_t status;
  flash_param_status_t watchdog_status;
  const size_t commit_offset = offsetof(flash_param_image_t, commit_marker);

  if (FlashParam_BundleValid(bundle) == 0U)
  {
    return FLASH_PARAM_STATUS_INVALID;
  }
  active = FlashParam_SelectActiveSlot(&active_sequence);
  target = (active == 0) ? 1U : 0U;
  status = FlashParam_EncodeBundle(bundle, active_sequence + 1UL, image, sizeof(image));
  if (status != FLASH_PARAM_STATUS_OK)
  {
    return status;
  }
  status = FlashParam_WatchdogEnterMaintenance();
  if (status != FLASH_PARAM_STATUS_OK)
  {
    return status;
  }
  status = FlashParam_StorageUnlock();
  if (status != FLASH_PARAM_STATUS_OK)
  {
    (void)FlashParam_WatchdogExitMaintenance();
    return status;
  }
  status = FlashParam_EraseSlot(target);
  if (status == FLASH_PARAM_STATUS_OK)
  {
    for (size_t offset = 0U; offset < commit_offset; offset += 4U)
    {
      uint32_t word;
      memcpy(&word, &image[offset], sizeof(word));
      status = FlashParam_ProgramWord(target, offset, word);
      if (status != FLASH_PARAM_STATUS_OK)
      {
        break;
      }
    }
  }
  if (status == FLASH_PARAM_STATUS_OK)
  {
    uint32_t commit_word;
    memcpy(&commit_word, &image[commit_offset], sizeof(commit_word));
    status = FlashParam_ProgramWord(target, commit_offset, commit_word);
  }
  FlashParam_StorageLock();
  watchdog_status = FlashParam_WatchdogExitMaintenance();
  if (status == FLASH_PARAM_STATUS_OK && watchdog_status != FLASH_PARAM_STATUS_OK)
  {
    status = watchdog_status;
  }
  if (status != FLASH_PARAM_STATUS_OK)
  {
    return FLASH_PARAM_STATUS_WRITE_ERROR;
  }
  status = FlashParam_DecodeBundle(FlashParam_SlotData(target),
                                   FLASH_PARAM_IMAGE_SIZE,
                                   &verify_bundle,
                                   &verify_sequence);
  if (status != FLASH_PARAM_STATUS_OK || verify_sequence != active_sequence + 1UL ||
      memcmp(&verify_bundle, bundle, sizeof(*bundle)) != 0)
  {
    return FLASH_PARAM_STATUS_WRITE_ERROR;
  }
  return FLASH_PARAM_STATUS_OK;
}

flash_param_status_t FlashParam_Load(param_store_t *params)
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

flash_param_status_t FlashParam_Save(const param_store_t *params)
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
  flash_param_status_t status;
  flash_param_status_t watchdog_status;

  status = FlashParam_WatchdogEnterMaintenance();
  if (status != FLASH_PARAM_STATUS_OK)
  {
    return status;
  }
  status = FlashParam_StorageUnlock();
  if (status != FLASH_PARAM_STATUS_OK)
  {
    (void)FlashParam_WatchdogExitMaintenance();
    return status;
  }
  status = FlashParam_EraseSlot(0U);
  if (status == FLASH_PARAM_STATUS_OK)
  {
    status = FlashParam_EraseSlot(1U);
  }
  FlashParam_StorageLock();
  watchdog_status = FlashParam_WatchdogExitMaintenance();
  if (status == FLASH_PARAM_STATUS_OK && watchdog_status != FLASH_PARAM_STATUS_OK)
  {
    status = watchdog_status;
  }
  return status;
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
