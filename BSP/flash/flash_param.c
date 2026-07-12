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
  uint32_t version;
  float max_linear_mps;
  float max_angular_rps;
  float speed_ramp_mps2;
  float angular_ramp_rps2;
  float wheel_radius_m;
  float track_width_m;
  float pid_kp[MOTOR_ID_COUNT];
  float pid_ki[MOTOR_ID_COUNT];
  float pid_kd[MOTOR_ID_COUNT];
  float pid_integral_limit;
  int8_t motor_dir[MOTOR_ID_COUNT];
  int8_t encoder_dir[MOTOR_ID_COUNT];
  uint16_t current_zero_raw[MOTOR_ID_COUNT];
  uint8_t current_zero_valid;
  float imu_gyro_bias_dps[3];
  uint8_t imu_gyro_bias_valid;
} param_store_v1_t;

typedef struct
{
  param_store_v1_t params;
  imu_bmi270_calibration_t imu_calibration;
} flash_param_bundle_v2_t;

typedef struct
{
  uint32_t version;
  float max_linear_mps;
  float max_angular_rps;
  float speed_ramp_mps2;
  float angular_ramp_rps2;
  float wheel_radius_m;
  float track_width_m;
  float pid_kp[MOTOR_ID_COUNT];
  float pid_ki[MOTOR_ID_COUNT];
  float pid_kd[MOTOR_ID_COUNT];
  float pid_integral_limit;
  int8_t motor_dir[MOTOR_ID_COUNT];
  int8_t encoder_dir[MOTOR_ID_COUNT];
  uint16_t current_zero_raw[MOTOR_ID_COUNT];
  uint8_t current_zero_valid;
  float imu_gyro_bias_dps[3];
  uint8_t imu_gyro_bias_valid;
  uint16_t line_threshold_raw[PARAM_STORE_LINE_CHANNELS];
  uint8_t line_active_low;
  float line_kp;
  float line_kd;
  float line_speed_mps;
  float line_slowdown_gain;
  uint8_t line_detect_debounce_frames;
  uint8_t line_lost_debounce_frames;
  float current_observe_a[MOTOR_ID_COUNT];
  float current_soft_limit_a[MOTOR_ID_COUNT];
  float current_fault_a[MOTOR_ID_COUNT];
  uint16_t current_fault_debounce_ms;
  float straight_wheel_coupling_gain;
  float straight_heading_kp;
  uint8_t straight_heading_hold_enabled;
} param_store_v2_t;

typedef struct
{
  param_store_v2_t params;
  imu_bmi270_calibration_t imu_calibration;
} flash_param_bundle_v3_t;

typedef struct
{
  uint32_t magic;
  uint32_t schema_version;
  uint32_t sequence;
  uint32_t payload_size;
  flash_param_bundle_v2_t payload;
  uint32_t crc32;
  uint32_t commit_marker;
} flash_param_image_v2_t;

typedef struct
{
  uint32_t magic;
  uint32_t schema_version;
  uint32_t sequence;
  uint32_t payload_size;
  flash_param_bundle_v3_t payload;
  uint32_t crc32;
  uint32_t commit_marker;
} flash_param_image_v3_t;

typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t data_size;
  uint32_t crc32;
  param_store_v1_t data;
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

static void FlashParam_MigrateV1Params(const param_store_v1_t *old,
                                       param_store_t *params)
{
  ParamStore_Defaults(params);
  params->max_linear_mps = old->max_linear_mps;
  params->max_angular_rps = old->max_angular_rps;
  params->speed_ramp_mps2 = old->speed_ramp_mps2;
  params->angular_ramp_rps2 = old->angular_ramp_rps2;
  params->wheel_radius_m = old->wheel_radius_m;
  params->track_width_m = old->track_width_m;
  params->pid_integral_limit = old->pid_integral_limit;
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    params->pid_kp[i] = old->pid_kp[i];
    params->pid_ki[i] = old->pid_ki[i];
    params->pid_kd[i] = old->pid_kd[i];
    params->motor_dir[i] = old->motor_dir[i];
    params->encoder_dir[i] = old->encoder_dir[i];
    params->current_zero_raw[i] = old->current_zero_raw[i];
  }
  params->current_zero_valid = old->current_zero_valid;
}

static void FlashParam_MigrateV2Bundle(const flash_param_bundle_v2_t *old,
                                       flash_param_bundle_t *bundle)
{
  FlashParam_MigrateV1Params(&old->params, &bundle->params);
  bundle->imu_calibration = old->imu_calibration;
  if (ImuBmi270Calibration_Validate(&bundle->imu_calibration) == 0U)
  {
    ImuBmi270Calibration_Default(&bundle->imu_calibration);
  }
  if (old->params.imu_gyro_bias_valid != 0U &&
      bundle->imu_calibration.gyro_bias_dps[0] == 0.0f &&
      bundle->imu_calibration.gyro_bias_dps[1] == 0.0f &&
      bundle->imu_calibration.gyro_bias_dps[2] == 0.0f)
  {
    for (uint8_t i = 0U; i < 3U; ++i)
    {
      bundle->imu_calibration.gyro_bias_dps[i] = old->params.imu_gyro_bias_dps[i];
    }
    bundle->imu_calibration.crc = ImuBmi270Calibration_Crc(&bundle->imu_calibration);
  }
  bundle->params.imu_gyro_bias_valid = 0U;
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    bundle->params.imu_gyro_bias_dps[i] = 0.0f;
  }
}

static void FlashParam_MigrateV3Bundle(const flash_param_bundle_v3_t *old,
                                       flash_param_bundle_t *bundle)
{
  ParamStore_Defaults(&bundle->params);
  bundle->params.max_linear_mps = old->params.max_linear_mps;
  bundle->params.max_angular_rps = old->params.max_angular_rps;
  bundle->params.speed_ramp_mps2 = old->params.speed_ramp_mps2;
  bundle->params.angular_ramp_rps2 = old->params.angular_ramp_rps2;
  bundle->params.wheel_radius_m = old->params.wheel_radius_m;
  bundle->params.track_width_m = old->params.track_width_m;
  memcpy(bundle->params.pid_kp, old->params.pid_kp, sizeof(bundle->params.pid_kp));
  memcpy(bundle->params.pid_ki, old->params.pid_ki, sizeof(bundle->params.pid_ki));
  memcpy(bundle->params.pid_kd, old->params.pid_kd, sizeof(bundle->params.pid_kd));
  bundle->params.pid_integral_limit = old->params.pid_integral_limit;
  memcpy(bundle->params.motor_dir, old->params.motor_dir, sizeof(bundle->params.motor_dir));
  memcpy(bundle->params.encoder_dir, old->params.encoder_dir, sizeof(bundle->params.encoder_dir));
  memcpy(bundle->params.current_zero_raw, old->params.current_zero_raw,
         sizeof(bundle->params.current_zero_raw));
  bundle->params.current_zero_valid = old->params.current_zero_valid;
  memcpy(bundle->params.line_threshold_raw, old->params.line_threshold_raw,
         sizeof(bundle->params.line_threshold_raw));
  bundle->params.line_active_low = old->params.line_active_low;
  bundle->params.line_kp = old->params.line_kp;
  bundle->params.line_kd = old->params.line_kd;
  bundle->params.line_speed_mps = old->params.line_speed_mps;
  bundle->params.line_slowdown_gain = old->params.line_slowdown_gain;
  bundle->params.line_detect_debounce_frames = old->params.line_detect_debounce_frames;
  bundle->params.line_lost_debounce_frames = old->params.line_lost_debounce_frames;
  memcpy(bundle->params.current_observe_a, old->params.current_observe_a,
         sizeof(bundle->params.current_observe_a));
  memcpy(bundle->params.current_soft_limit_a, old->params.current_soft_limit_a,
         sizeof(bundle->params.current_soft_limit_a));
  memcpy(bundle->params.current_fault_a, old->params.current_fault_a,
         sizeof(bundle->params.current_fault_a));
  bundle->params.current_fault_debounce_ms = old->params.current_fault_debounce_ms;
  bundle->params.straight_wheel_coupling_gain = old->params.straight_wheel_coupling_gain;
  bundle->params.straight_heading_kp = old->params.straight_heading_kp;
  bundle->params.straight_heading_hold_enabled = 0U;
  bundle->imu_calibration = old->imu_calibration;
  if (ImuBmi270Calibration_Validate(&bundle->imu_calibration) == 0U)
  {
    ImuBmi270Calibration_Default(&bundle->imu_calibration);
  }
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
  encoded.payload.params.imu_gyro_bias_valid = 0U;
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    encoded.payload.params.imu_gyro_bias_dps[i] = 0.0f;
  }
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

  if (image == 0 || bundle == 0 || image_size < (sizeof(uint32_t) * 2U))
  {
    return FLASH_PARAM_STATUS_INVALID;
  }
  if (FlashParam_IsBlank(image, image_size) != 0U)
  {
    return FLASH_PARAM_STATUS_EMPTY;
  }
  encoded = (const flash_param_image_t *)image;
  if (encoded->magic == FLASH_PARAM_MAGIC && encoded->schema_version == 2UL)
  {
    const flash_param_image_v2_t *old = (const flash_param_image_v2_t *)image;
    uint32_t old_crc;

    if (image_size < sizeof(*old) || old->payload_size != sizeof(old->payload) ||
        old->commit_marker != FLASH_PARAM_COMMIT_MARKER)
    {
      return FLASH_PARAM_STATUS_UNSUPPORTED;
    }
    old_crc = FlashParam_Crc32((const uint8_t *)old,
                               offsetof(flash_param_image_v2_t, crc32));
    if (old_crc != old->crc32)
    {
      return FLASH_PARAM_STATUS_CRC_ERROR;
    }
    FlashParam_MigrateV2Bundle(&old->payload, bundle);
    if (ParamStore_Validate(&bundle->params) == 0U)
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
    uint32_t old_crc;

    if (image_size < sizeof(*old) || old->payload_size != sizeof(old->payload) ||
        old->commit_marker != FLASH_PARAM_COMMIT_MARKER)
    {
      return FLASH_PARAM_STATUS_UNSUPPORTED;
    }
    old_crc = FlashParam_Crc32((const uint8_t *)old,
                               offsetof(flash_param_image_v3_t, crc32));
    if (old_crc != old->crc32)
    {
      return FLASH_PARAM_STATUS_CRC_ERROR;
    }
    FlashParam_MigrateV3Bundle(&old->payload, bundle);
    if (ParamStore_Validate(&bundle->params) == 0U)
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
      legacy->version != 1UL ||
      legacy->data_size != sizeof(param_store_v1_t))
  {
    return FLASH_PARAM_STATUS_UNSUPPORTED;
  }
  crc = FlashParam_Crc32((const uint8_t *)&legacy->data, sizeof(legacy->data));
  if (crc != legacy->crc32)
  {
    return FLASH_PARAM_STATUS_CRC_ERROR;
  }
  FlashParam_MigrateV1Params(&legacy->data, &bundle->params);
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
  legacy.version = 1UL;
  legacy.data_size = sizeof(legacy.data);
  legacy.data.version = 1UL;
  legacy.data.max_linear_mps = params->max_linear_mps;
  legacy.data.max_angular_rps = params->max_angular_rps;
  legacy.data.speed_ramp_mps2 = params->speed_ramp_mps2;
  legacy.data.angular_ramp_rps2 = params->angular_ramp_rps2;
  legacy.data.wheel_radius_m = params->wheel_radius_m;
  legacy.data.track_width_m = params->track_width_m;
  legacy.data.pid_integral_limit = params->pid_integral_limit;
  memcpy(legacy.data.pid_kp, params->pid_kp, sizeof(legacy.data.pid_kp));
  memcpy(legacy.data.pid_ki, params->pid_ki, sizeof(legacy.data.pid_ki));
  memcpy(legacy.data.pid_kd, params->pid_kd, sizeof(legacy.data.pid_kd));
  memcpy(legacy.data.motor_dir, params->motor_dir, sizeof(legacy.data.motor_dir));
  memcpy(legacy.data.encoder_dir, params->encoder_dir, sizeof(legacy.data.encoder_dir));
  memcpy(legacy.data.current_zero_raw, params->current_zero_raw, sizeof(legacy.data.current_zero_raw));
  legacy.data.current_zero_valid = params->current_zero_valid;
  legacy.crc32 = FlashParam_Crc32((const uint8_t *)&legacy.data, sizeof(legacy.data));
  memset(flash_param_host_slots[1], 0xFF, FLASH_PARAM_IMAGE_SIZE);
  memcpy(flash_param_host_slots[1], &legacy, sizeof(legacy));
  return FLASH_PARAM_STATUS_OK;
}

flash_param_status_t FlashParamHost_SeedSchema2WithCalibration(
  const param_store_t *params, const imu_bmi270_calibration_t *calibration)
{
  flash_param_image_v2_t image = {0};

  if (ParamStore_Validate(params) == 0U)
  {
    return FLASH_PARAM_STATUS_INVALID;
  }
  image.magic = FLASH_PARAM_MAGIC;
  image.schema_version = 2UL;
  image.sequence = 7UL;
  image.payload_size = sizeof(image.payload);
  image.payload.params.version = 1UL;
  image.payload.params.max_linear_mps = params->max_linear_mps;
  image.payload.params.max_angular_rps = params->max_angular_rps;
  image.payload.params.speed_ramp_mps2 = params->speed_ramp_mps2;
  image.payload.params.angular_ramp_rps2 = params->angular_ramp_rps2;
  image.payload.params.wheel_radius_m = params->wheel_radius_m;
  image.payload.params.track_width_m = params->track_width_m;
  image.payload.params.pid_integral_limit = params->pid_integral_limit;
  memcpy(image.payload.params.pid_kp, params->pid_kp, sizeof(image.payload.params.pid_kp));
  memcpy(image.payload.params.pid_ki, params->pid_ki, sizeof(image.payload.params.pid_ki));
  memcpy(image.payload.params.pid_kd, params->pid_kd, sizeof(image.payload.params.pid_kd));
  memcpy(image.payload.params.motor_dir, params->motor_dir, sizeof(image.payload.params.motor_dir));
  memcpy(image.payload.params.encoder_dir, params->encoder_dir, sizeof(image.payload.params.encoder_dir));
  memcpy(image.payload.params.current_zero_raw, params->current_zero_raw,
         sizeof(image.payload.params.current_zero_raw));
  image.payload.params.current_zero_valid = params->current_zero_valid;
  memcpy(image.payload.params.imu_gyro_bias_dps, params->imu_gyro_bias_dps,
         sizeof(image.payload.params.imu_gyro_bias_dps));
  image.payload.params.imu_gyro_bias_valid = params->imu_gyro_bias_valid;
  if (calibration != 0)
  {
    image.payload.imu_calibration = *calibration;
  }
  else
  {
    ImuBmi270Calibration_Default(&image.payload.imu_calibration);
  }
  image.crc32 = FlashParam_Crc32((const uint8_t *)&image,
                                 offsetof(flash_param_image_v2_t, crc32));
  image.commit_marker = FLASH_PARAM_COMMIT_MARKER;
  memset(flash_param_host_slots[0], 0xFF, FLASH_PARAM_IMAGE_SIZE);
  memcpy(flash_param_host_slots[0], &image, sizeof(image));
  return FLASH_PARAM_STATUS_OK;
}

flash_param_status_t FlashParamHost_SeedSchema2(const param_store_t *params)
{
  return FlashParamHost_SeedSchema2WithCalibration(params, 0);
}

flash_param_status_t FlashParamHost_SeedSchema3(const param_store_t *params)
{
  flash_param_image_v3_t image = {0};

  if (ParamStore_Validate(params) == 0U)
  {
    return FLASH_PARAM_STATUS_INVALID;
  }
  image.magic = FLASH_PARAM_MAGIC;
  image.schema_version = 3UL;
  image.sequence = 8UL;
  image.payload_size = sizeof(image.payload);
  image.payload.params.version = 2UL;
  image.payload.params.max_linear_mps = params->max_linear_mps;
  image.payload.params.max_angular_rps = params->max_angular_rps;
  image.payload.params.speed_ramp_mps2 = params->speed_ramp_mps2;
  image.payload.params.angular_ramp_rps2 = params->angular_ramp_rps2;
  image.payload.params.wheel_radius_m = params->wheel_radius_m;
  image.payload.params.track_width_m = params->track_width_m;
  memcpy(image.payload.params.pid_kp, params->pid_kp, sizeof(image.payload.params.pid_kp));
  memcpy(image.payload.params.pid_ki, params->pid_ki, sizeof(image.payload.params.pid_ki));
  memcpy(image.payload.params.pid_kd, params->pid_kd, sizeof(image.payload.params.pid_kd));
  image.payload.params.pid_integral_limit = params->pid_integral_limit;
  memcpy(image.payload.params.motor_dir, params->motor_dir, sizeof(image.payload.params.motor_dir));
  memcpy(image.payload.params.encoder_dir, params->encoder_dir, sizeof(image.payload.params.encoder_dir));
  memcpy(image.payload.params.current_zero_raw, params->current_zero_raw,
         sizeof(image.payload.params.current_zero_raw));
  image.payload.params.current_zero_valid = params->current_zero_valid;
  memcpy(image.payload.params.line_threshold_raw, params->line_threshold_raw,
         sizeof(image.payload.params.line_threshold_raw));
  image.payload.params.line_active_low = params->line_active_low;
  image.payload.params.line_kp = params->line_kp;
  image.payload.params.line_kd = params->line_kd;
  image.payload.params.line_speed_mps = params->line_speed_mps;
  image.payload.params.line_slowdown_gain = params->line_slowdown_gain;
  image.payload.params.line_detect_debounce_frames = params->line_detect_debounce_frames;
  image.payload.params.line_lost_debounce_frames = params->line_lost_debounce_frames;
  memcpy(image.payload.params.current_observe_a, params->current_observe_a,
         sizeof(image.payload.params.current_observe_a));
  memcpy(image.payload.params.current_soft_limit_a, params->current_soft_limit_a,
         sizeof(image.payload.params.current_soft_limit_a));
  memcpy(image.payload.params.current_fault_a, params->current_fault_a,
         sizeof(image.payload.params.current_fault_a));
  image.payload.params.current_fault_debounce_ms = params->current_fault_debounce_ms;
  image.payload.params.straight_wheel_coupling_gain = params->straight_wheel_coupling_gain;
  image.payload.params.straight_heading_kp = params->straight_heading_kp;
  image.payload.params.straight_heading_hold_enabled = params->straight_heading_hold_enabled;
  ImuBmi270Calibration_Default(&image.payload.imu_calibration);
  image.crc32 = FlashParam_Crc32((const uint8_t *)&image,
                                 offsetof(flash_param_image_v3_t, crc32));
  image.commit_marker = FLASH_PARAM_COMMIT_MARKER;
  memset(flash_param_host_slots[0], 0xFF, FLASH_PARAM_IMAGE_SIZE);
  memcpy(flash_param_host_slots[0], &image, sizeof(image));
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
