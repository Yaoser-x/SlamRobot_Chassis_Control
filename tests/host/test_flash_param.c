#include "flash_param.h"
#include "param_store.h"

#include <stdio.h>
#include <string.h>

static void require_int(int condition, const char *message)
{
  if (!condition)
  {
    (void)printf("FAIL: %s\n", message);
    __builtin_exit(1);
  }
}

static void make_bundle(flash_param_bundle_t *bundle, float max_linear_mps)
{
  ParamStore_Defaults(&bundle->params);
  bundle->params.max_linear_mps = max_linear_mps;
  ImuBmi270Calibration_Default(&bundle->imu_calibration);
}

static void test_codec_roundtrips_valid_params(void)
{
  uint8_t image[FLASH_PARAM_IMAGE_SIZE];
  param_store_t input;
  param_store_t output;

  ParamStore_Defaults(&input);
  input.max_linear_mps = 0.33f;
  input.imu_gyro_bias_valid = 1U;
  input.imu_gyro_bias_dps[2] = 0.12f;

  require_int(FlashParam_Encode(&input, image, sizeof(image)) == FLASH_PARAM_STATUS_OK,
              "encode succeeds");
  memset(&output, 0, sizeof(output));
  require_int(FlashParam_Decode(image, sizeof(image), &output) == FLASH_PARAM_STATUS_OK,
              "decode succeeds");
  require_int(output.version == PARAM_STORE_VERSION, "decoded version");
  require_int(output.max_linear_mps > 0.32f && output.max_linear_mps < 0.34f,
              "decoded float field");
  require_int(output.imu_gyro_bias_valid == 1U, "decoded calibration flag");
}

static void test_empty_and_corrupt_images_fall_back(void)
{
  uint8_t image[FLASH_PARAM_IMAGE_SIZE];
  param_store_t output;

  memset(image, 0xFF, sizeof(image));
  require_int(FlashParam_Decode(image, sizeof(image), &output) == FLASH_PARAM_STATUS_EMPTY,
              "blank flash is empty");

  ParamStore_Defaults(&output);
  require_int(FlashParam_Encode(&output, image, sizeof(image)) == FLASH_PARAM_STATUS_OK,
              "encode before corruption succeeds");
  image[sizeof(image) - 1U] ^= 0x5AU;
  require_int(FlashParam_Decode(image, sizeof(image), &output) == FLASH_PARAM_STATUS_CRC_ERROR,
              "crc catches image corruption");
}

static void test_ab_update_survives_every_interrupted_word(void)
{
  flash_param_bundle_t old_bundle;
  flash_param_bundle_t new_bundle;
  flash_param_bundle_t loaded;

  make_bundle(&old_bundle, 0.30f);
  make_bundle(&new_bundle, 0.60f);
  for (int32_t cut = 0; cut < (int32_t)FLASH_PARAM_IMAGE_WORD_COUNT; ++cut)
  {
    FlashParamHost_Reset();
    require_int(FlashParam_SaveBundle(&old_bundle) == FLASH_PARAM_STATUS_OK,
                "initial A copy saves");
    FlashParamHost_SetProgramBudget(cut);
    require_int(FlashParam_SaveBundle(&new_bundle) == FLASH_PARAM_STATUS_WRITE_ERROR,
                "interrupted B write reports failure");
    FlashParamHost_SetProgramBudget(-1);
    require_int(FlashParam_LoadBundle(&loaded) == FLASH_PARAM_STATUS_OK,
                "old copy remains loadable after interrupted update");
    require_int(loaded.params.max_linear_mps > 0.29f &&
                loaded.params.max_linear_mps < 0.31f,
                "interrupted update selects old complete copy");
  }
  FlashParamHost_Reset();
  require_int(FlashParam_SaveBundle(&old_bundle) == FLASH_PARAM_STATUS_OK,
              "initial copy saves before complete update");
  FlashParamHost_SetProgramBudget((int32_t)FLASH_PARAM_IMAGE_WORD_COUNT);
  require_int(FlashParam_SaveBundle(&new_bundle) == FLASH_PARAM_STATUS_OK,
              "complete B update commits");
  FlashParamHost_SetProgramBudget(-1);
  require_int(FlashParam_LoadBundle(&loaded) == FLASH_PARAM_STATUS_OK,
              "new committed copy loads");
  require_int(loaded.params.max_linear_mps > 0.59f &&
              loaded.params.max_linear_mps < 0.61f,
              "newest committed sequence wins");
}

static void test_legacy_v1_migrates_to_sector6(void)
{
  param_store_t legacy;
  flash_param_bundle_t loaded;
  flash_param_bundle_t migrated;

  ParamStore_Defaults(&legacy);
  legacy.max_linear_mps = 0.41f;
  FlashParamHost_Reset();
  require_int(FlashParamHost_SeedLegacy(&legacy) == FLASH_PARAM_STATUS_OK,
              "legacy sector7 image seeded");
  require_int(FlashParam_LoadBundle(&loaded) == FLASH_PARAM_STATUS_OK,
              "legacy sector7 image loads");
  require_int(loaded.params.max_linear_mps > 0.40f &&
              loaded.params.max_linear_mps < 0.42f,
              "legacy parameters preserved in RAM");
  make_bundle(&migrated, 0.51f);
  require_int(FlashParam_SaveBundle(&migrated) == FLASH_PARAM_STATUS_OK,
              "first v2 save migrates to inactive sector6");
  require_int(FlashParam_LoadBundle(&loaded) == FLASH_PARAM_STATUS_OK,
              "migrated v2 image loads");
  require_int(loaded.params.max_linear_mps > 0.50f &&
              loaded.params.max_linear_mps < 0.52f,
              "migrated v2 parameters win");
}

static void test_flash_maintenance_brackets_watchdog_on_all_paths(void)
{
  flash_param_bundle_t bundle;

  make_bundle(&bundle, 0.55f);

  FlashParamHost_Reset();
  require_int(FlashParam_SaveBundle(&bundle) == FLASH_PARAM_STATUS_OK,
              "watchdog policy success save completes");
  require_int(FlashParamHost_GetWatchdogMaintenanceEnterCount() == 1U,
              "successful save enters long watchdog window once");
  require_int(FlashParamHost_GetWatchdogMaintenanceExitCount() == 1U,
              "successful save restores normal watchdog window once");

  FlashParamHost_SetProgramBudget(0);
  require_int(FlashParam_SaveBundle(&bundle) == FLASH_PARAM_STATUS_WRITE_ERROR,
              "watchdog policy observes injected write failure");
  require_int(FlashParamHost_GetWatchdogMaintenanceEnterCount() == 2U,
              "failed save enters long watchdog window");
  require_int(FlashParamHost_GetWatchdogMaintenanceExitCount() == 2U,
              "failed save still restores normal watchdog window");

  require_int(FlashParam_Erase() == FLASH_PARAM_STATUS_OK,
              "watchdog policy erase completes");
  require_int(FlashParamHost_GetWatchdogMaintenanceEnterCount() == 3U,
              "erase enters long watchdog window");
  require_int(FlashParamHost_GetWatchdogMaintenanceExitCount() == 3U,
              "erase restores normal watchdog window");
}

int main(void)
{
  test_codec_roundtrips_valid_params();
  test_empty_and_corrupt_images_fall_back();
  test_ab_update_survives_every_interrupted_word();
  test_legacy_v1_migrates_to_sector6();
  test_flash_maintenance_brackets_watchdog_on_all_paths();

  (void)printf("PASS: flash param host tests\n");
  return 0;
}
