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

int main(void)
{
  test_codec_roundtrips_valid_params();
  test_empty_and_corrupt_images_fall_back();

  (void)printf("PASS: flash param host tests\n");
  return 0;
}
