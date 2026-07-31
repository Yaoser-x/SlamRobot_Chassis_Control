#include "flash_parameter_image.h"
#include "motor_types.h"
#include "param_service.h"
#include "parameter_imu_calibration_types.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

_Static_assert(sizeof(flash_param_bundle_t) == 340U, "Beta4 bundle layout changed");
_Static_assert(FLASH_PARAM_IMAGE_SIZE == 364U, "Beta4 image layout changed");

static uint32_t fake_primask;

uint32_t __get_PRIMASK(void)
{
    return fake_primask;
}

void __disable_irq(void)
{
    fake_primask = 1U;
}

void __set_PRIMASK(uint32_t primask)
{
    fake_primask = primask;
}

static void require_int(int condition, const char *message)
{
    if (!condition)
    {
        (void)printf("FAIL: %s\n", message);
        exit(1);
    }
}

static void make_bundle(flash_param_bundle_t *bundle, float max_linear_mps)
{
    memset(bundle, 0, sizeof(*bundle));
    ParamService_Defaults(&bundle->params);
    bundle->params.max_linear_mps = max_linear_mps;
    ParameterImuCalibration_Default(&bundle->imu_calibration);
}

static void load_hex_fixture(const char *path, uint8_t *image, size_t image_size)
{
    FILE *fixture = fopen(path, "rb");
    require_int(fixture != NULL, "schema4 golden fixture opens");
    for (size_t index = 0U; index < image_size; ++index)
    {
        unsigned int byte = 0U;
        require_int(fscanf(fixture, "%2x", &byte) == 1, "schema4 golden has every byte");
        image[index] = (uint8_t)byte;
    }
    require_int(fscanf(fixture, " %*c") == EOF, "schema4 golden has exact image length");
    (void)fclose(fixture);
}

static void test_schema4_matches_beta4_golden(const char *fixture_path)
{
    uint8_t              encoded[FLASH_PARAM_IMAGE_SIZE];
    uint8_t              golden[FLASH_PARAM_IMAGE_SIZE];
    flash_param_bundle_t bundle;
    flash_param_bundle_t decoded;
    uint32_t             sequence = 0U;

    make_bundle(&bundle, 0.375f);
    bundle.params.pid_kd[MOTOR_ID_M1] = 0.05f;
    bundle.params.pid_kd[MOTOR_ID_M2] = 0.15f;
    bundle.params.pid_kd[MOTOR_ID_M3] = 0.18f;
    require_int(FlashParam_EncodeBundle(&bundle, 0x10203040UL, encoded, sizeof(encoded)) == FLASH_PARAM_STATUS_OK,
                "schema4 fixed-sequence image encodes");
    load_hex_fixture(fixture_path, golden, sizeof(golden));
    require_int(memcmp(encoded, golden, sizeof(encoded)) == 0, "schema4 bytes match Beta4 golden");

    memset(&decoded, 0, sizeof(decoded));
    require_int(FlashParam_DecodeBundle(golden, sizeof(golden), &decoded, &sequence) == FLASH_PARAM_STATUS_OK,
                "Beta4 golden image decodes");
    require_int(sequence == 0x10203040UL, "Beta4 golden sequence is preserved");
    require_int(memcmp(&decoded, &bundle, sizeof(bundle)) == 0, "Beta4 golden payload is preserved");
}

static void test_codec_roundtrips_valid_params(void)
{
    uint8_t       image[FLASH_PARAM_IMAGE_SIZE];
    param_model_t input;
    param_model_t output;

    ParamService_Defaults(&input);
    input.max_linear_mps                = 0.33f;
    input.straight_trim_reverse_030_mps = -0.043f;
    input.straight_heading_ki           = 0.001f;
    input.imu_gyro_bias_valid           = 1U;
    input.imu_gyro_bias_dps[2]          = 0.12f;

    require_int(FlashParam_Encode(&input, image, sizeof(image)) == FLASH_PARAM_STATUS_OK, "encode succeeds");
    memset(&output, 0, sizeof(output));
    require_int(FlashParam_Decode(image, sizeof(image), &output) == FLASH_PARAM_STATUS_OK, "decode succeeds");
    require_int(output.version == PARAM_SERVICE_VERSION, "decoded version");
    require_int(output.max_linear_mps > 0.32f && output.max_linear_mps < 0.34f, "decoded float field");
    require_int(output.imu_gyro_bias_valid == 0U, "schema3 removes duplicate param bias");
    require_int(output.straight_trim_reverse_030_mps < -0.042f && output.straight_trim_reverse_030_mps > -0.044f,
                "schema4 roundtrips directional trim");
    require_int(output.straight_heading_ki > 0.0009f && output.straight_heading_ki < 0.0011f,
                "schema4 roundtrips heading ki");
}

static void test_schema3_migrates_explicit_old_layout(void)
{
    param_model_t        old_params;
    flash_param_bundle_t loaded;

    ParamService_Defaults(&old_params);
    old_params.track_width_m                 = 0.177f;
    old_params.line_kp                       = 0.73f;
    old_params.current_fault_a[MOTOR_ID_M2]  = 3.2f;
    old_params.straight_wheel_coupling_gain  = 0.41f;
    old_params.straight_heading_kp           = 0.018f;
    old_params.straight_heading_hold_enabled = 1U;
    FlashParamHost_Reset();
    require_int(FlashParamHost_SeedSchema3(&old_params) == FLASH_PARAM_STATUS_OK,
                "schema3 image seeded with explicit old layout");
    require_int(FlashParam_LoadBundle(&loaded) == FLASH_PARAM_STATUS_OK, "schema3 image migrates to schema4");
    require_int(loaded.params.version == 3UL, "schema3 migration upgrades ParamService version");
    require_int(loaded.params.track_width_m > 0.176f && loaded.params.track_width_m < 0.178f,
                "schema3 migration preserves geometry");
    require_int(loaded.params.line_kp > 0.72f && loaded.params.line_kp < 0.74f,
                "schema3 migration preserves line parameters");
    require_int(loaded.params.current_fault_a[MOTOR_ID_M2] > 3.1f, "schema3 migration preserves current parameters");
    require_int(loaded.params.straight_wheel_coupling_gain > 0.40f, "schema3 migration preserves coupling");
    require_int(loaded.params.straight_heading_kp > 0.017f, "schema3 migration preserves old Kp for supervised reuse");
    require_int(loaded.params.straight_heading_hold_enabled == 0U,
                "schema3 migration forces new heading controller off");
    require_int(loaded.params.straight_trim_forward_015_mps == 0.0f
                    && loaded.params.straight_trim_forward_030_mps == 0.0f
                    && loaded.params.straight_trim_reverse_015_mps == 0.0f
                    && loaded.params.straight_trim_reverse_030_mps == 0.0f,
                "schema3 migration initializes all trims to zero");
    require_int(loaded.params.straight_heading_ki == 0.0f, "schema3 migration initializes Ki to zero");
    require_int(loaded.params.straight_max_speed_mps > 0.29f && loaded.params.straight_max_speed_mps < 0.31f,
                "schema3 migration initializes safe maximum speed");
}

static void test_schema2_migrates_bias_to_calibration_once(void)
{
    param_model_t        old_params;
    flash_param_bundle_t loaded;

    ParamService_Defaults(&old_params);
    old_params.max_linear_mps       = 0.44f;
    old_params.imu_gyro_bias_valid  = 1U;
    old_params.imu_gyro_bias_dps[0] = 0.11f;
    old_params.imu_gyro_bias_dps[1] = -0.22f;
    old_params.imu_gyro_bias_dps[2] = 0.33f;
    FlashParamHost_Reset();
    require_int(FlashParamHost_SeedSchema2(&old_params) == FLASH_PARAM_STATUS_OK, "schema2 image seeded");
    require_int(FlashParam_LoadBundle(&loaded) == FLASH_PARAM_STATUS_OK, "schema2 image migrates");
    require_int(loaded.params.version == PARAM_SERVICE_VERSION, "schema2 migration upgrades param version");
    require_int(loaded.params.imu_gyro_bias_valid == 0U, "schema2 migration clears duplicate param bias");
    require_int(loaded.imu_calibration.gyro_bias_dps[0] > 0.10f && loaded.imu_calibration.gyro_bias_dps[0] < 0.12f,
                "schema2 migration moves bias to calibration owner");
    require_int(FlashParam_LoadBundle(&loaded) == FLASH_PARAM_STATUS_OK
                    && loaded.imu_calibration.gyro_bias_dps[0] > 0.10f
                    && loaded.imu_calibration.gyro_bias_dps[0] < 0.12f,
                "repeated load does not repeat or alter migration");
}

static void test_schema2_calibration_bias_has_priority(void)
{
    param_model_t        old_params;
    flash_param_bundle_t loaded;
    imu_calibration_t    calibration;

    ParamService_Defaults(&old_params);
    old_params.imu_gyro_bias_valid  = 1U;
    old_params.imu_gyro_bias_dps[0] = 0.11f;
    ParameterImuCalibration_Default(&calibration);
    calibration.gyro_bias_dps[0] = 0.77f;
    calibration.crc              = ParameterImuCalibration_Crc(&calibration);
    FlashParamHost_Reset();
    require_int(FlashParamHost_SeedSchema2WithCalibration(&old_params, &calibration) == FLASH_PARAM_STATUS_OK,
                "schema2 dual-bias image seeded");
    require_int(FlashParam_LoadBundle(&loaded) == FLASH_PARAM_STATUS_OK, "schema2 dual-bias image loads");
    require_int(loaded.imu_calibration.gyro_bias_dps[0] > 0.76f && loaded.imu_calibration.gyro_bias_dps[0] < 0.78f,
                "valid calibration bias wins over old ParamService bias");
}

static void test_empty_and_corrupt_images_fall_back(void)
{
    uint8_t       image[FLASH_PARAM_IMAGE_SIZE];
    param_model_t output;

    memset(image, 0xFF, sizeof(image));
    require_int(FlashParam_Decode(image, sizeof(image), &output) == FLASH_PARAM_STATUS_EMPTY, "blank flash is empty");

    ParamService_Defaults(&output);
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
        require_int(FlashParam_SaveBundle(&old_bundle) == FLASH_PARAM_STATUS_OK, "initial A copy saves");
        FlashParamHost_SetProgramBudget(cut);
        require_int(FlashParam_SaveBundle(&new_bundle) == FLASH_PARAM_STATUS_WRITE_ERROR,
                    "interrupted B write reports failure");
        FlashParamHost_SetProgramBudget(-1);
        require_int(FlashParam_LoadBundle(&loaded) == FLASH_PARAM_STATUS_OK,
                    "old copy remains loadable after interrupted update");
        require_int(loaded.params.max_linear_mps > 0.29f && loaded.params.max_linear_mps < 0.31f,
                    "interrupted update selects old complete copy");
    }
    FlashParamHost_Reset();
    require_int(FlashParam_SaveBundle(&old_bundle) == FLASH_PARAM_STATUS_OK,
                "initial copy saves before complete update");
    FlashParamHost_SetProgramBudget((int32_t)FLASH_PARAM_IMAGE_WORD_COUNT);
    require_int(FlashParam_SaveBundle(&new_bundle) == FLASH_PARAM_STATUS_OK, "complete B update commits");
    FlashParamHost_SetProgramBudget(-1);
    require_int(FlashParam_LoadBundle(&loaded) == FLASH_PARAM_STATUS_OK, "new committed copy loads");
    require_int(loaded.params.max_linear_mps > 0.59f && loaded.params.max_linear_mps < 0.61f,
                "newest committed sequence wins");
}

static void test_legacy_v1_migrates_to_sector6(void)
{
    param_model_t        legacy;
    flash_param_bundle_t loaded;
    flash_param_bundle_t migrated;

    ParamService_Defaults(&legacy);
    legacy.max_linear_mps = 0.41f;
    FlashParamHost_Reset();
    require_int(FlashParamHost_SeedLegacy(&legacy) == FLASH_PARAM_STATUS_OK, "legacy sector7 image seeded");
    require_int(FlashParam_LoadBundle(&loaded) == FLASH_PARAM_STATUS_OK, "legacy sector7 image loads");
    require_int(loaded.params.max_linear_mps > 0.40f && loaded.params.max_linear_mps < 0.42f,
                "legacy parameters preserved in RAM");
    make_bundle(&migrated, 0.51f);
    require_int(FlashParam_SaveBundle(&migrated) == FLASH_PARAM_STATUS_OK,
                "first v2 save migrates to inactive sector6");
    require_int(FlashParam_LoadBundle(&loaded) == FLASH_PARAM_STATUS_OK, "migrated v2 image loads");
    require_int(loaded.params.max_linear_mps > 0.50f && loaded.params.max_linear_mps < 0.52f,
                "migrated v2 parameters win");
}

static void test_flash_maintenance_brackets_watchdog_on_all_paths(void)
{
    flash_param_bundle_t bundle;

    make_bundle(&bundle, 0.55f);

    FlashParamHost_Reset();
    require_int(FlashParam_SaveBundle(&bundle) == FLASH_PARAM_STATUS_OK, "watchdog policy success save completes");
    require_int(FlashParamHost_GetWatchdogMaintenanceEnterCount() == 1U,
                "successful save enters long watchdog window once");
    require_int(FlashParamHost_GetWatchdogMaintenanceExitCount() == 1U,
                "successful save restores normal watchdog window once");

    FlashParamHost_SetProgramBudget(0);
    require_int(FlashParam_SaveBundle(&bundle) == FLASH_PARAM_STATUS_WRITE_ERROR,
                "watchdog policy observes injected write failure");
    require_int(FlashParamHost_GetWatchdogMaintenanceEnterCount() == 2U, "failed save enters long watchdog window");
    require_int(FlashParamHost_GetWatchdogMaintenanceExitCount() == 2U,
                "failed save still restores normal watchdog window");

    require_int(FlashParam_Erase() == FLASH_PARAM_STATUS_OK, "watchdog policy erase completes");
    require_int(FlashParamHost_GetWatchdogMaintenanceEnterCount() == 3U, "erase enters long watchdog window");
    require_int(FlashParamHost_GetWatchdogMaintenanceExitCount() == 3U, "erase restores normal watchdog window");
}

int main(int argc, char **argv)
{
    require_int(argc == 2, "schema4 golden fixture path is provided");
    test_schema4_matches_beta4_golden(argv[1]);
    test_codec_roundtrips_valid_params();
    test_empty_and_corrupt_images_fall_back();
    test_schema2_migrates_bias_to_calibration_once();
    test_schema2_calibration_bias_has_priority();
    test_schema3_migrates_explicit_old_layout();
    test_ab_update_survives_every_interrupted_word();
    test_legacy_v1_migrates_to_sector6();
    test_flash_maintenance_brackets_watchdog_on_all_paths();

    (void)printf("PASS: flash param host tests\n");
    return 0;
}
