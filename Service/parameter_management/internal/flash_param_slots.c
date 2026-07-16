#include "flash_param_slots.h"

#include "flash_param_codec.h"
#include "flash_param_schema.h"
#include "flash_storage.h"
#include "parameter_management_service.h"

#include <stddef.h>
#include <string.h>

#define FLASH_PARAM_SLOT_COUNT 2U

#ifdef FLASH_PARAM_HOST_TEST
static uint8_t  flash_param_host_slots[FLASH_PARAM_SLOT_COUNT][FLASH_PARAM_IMAGE_SIZE];
static int32_t  flash_param_host_program_budget = -1;
static uint32_t flash_param_host_watchdog_enter_count;
static uint32_t flash_param_host_watchdog_exit_count;

void FlashParamHost_Reset(void)
{
    memset(flash_param_host_slots, 0xFF, sizeof(flash_param_host_slots));
    flash_param_host_program_budget       = -1;
    flash_param_host_watchdog_enter_count = 0UL;
    flash_param_host_watchdog_exit_count  = 0UL;
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

static flash_param_status_t FlashParamSlots_EraseSlot(uint8_t slot)
{
    memset(flash_param_host_slots[slot], 0xFF, FLASH_PARAM_IMAGE_SIZE);
    return FLASH_PARAM_STATUS_OK;
}

static flash_param_status_t FlashParam_ProgramWord(uint8_t slot, size_t offset, uint32_t word)
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

flash_param_status_t FlashParamHost_SeedLegacy(const param_model_t *params)
{
    flash_param_legacy_image_t legacy;

    if (ParameterManagement_Validate(params) == 0U)
    {
        return FLASH_PARAM_STATUS_INVALID;
    }
    legacy                         = (flash_param_legacy_image_t){0};
    legacy.magic                   = FLASH_PARAM_MAGIC;
    legacy.version                 = 1UL;
    legacy.data_size               = sizeof(legacy.data);
    legacy.data.version            = 1UL;
    legacy.data.max_linear_mps     = params->max_linear_mps;
    legacy.data.max_angular_rps    = params->max_angular_rps;
    legacy.data.speed_ramp_mps2    = params->speed_ramp_mps2;
    legacy.data.angular_ramp_rps2  = params->angular_ramp_rps2;
    legacy.data.wheel_radius_m     = params->wheel_radius_m;
    legacy.data.track_width_m      = params->track_width_m;
    legacy.data.pid_integral_limit = params->pid_integral_limit;
    memcpy(legacy.data.pid_kp, params->pid_kp, sizeof(legacy.data.pid_kp));
    memcpy(legacy.data.pid_ki, params->pid_ki, sizeof(legacy.data.pid_ki));
    memcpy(legacy.data.pid_kd, params->pid_kd, sizeof(legacy.data.pid_kd));
    memcpy(legacy.data.motor_dir, params->motor_dir, sizeof(legacy.data.motor_dir));
    memcpy(legacy.data.encoder_dir, params->encoder_dir, sizeof(legacy.data.encoder_dir));
    memcpy(legacy.data.current_zero_raw, params->current_zero_raw, sizeof(legacy.data.current_zero_raw));
    legacy.data.current_zero_valid = params->current_zero_valid;
    legacy.crc32                   = FlashParamCodec_Crc32((const uint8_t *)&legacy.data, sizeof(legacy.data));
    memset(flash_param_host_slots[1], 0xFF, FLASH_PARAM_IMAGE_SIZE);
    memcpy(flash_param_host_slots[1], &legacy, sizeof(legacy));
    return FLASH_PARAM_STATUS_OK;
}

flash_param_status_t FlashParamHost_SeedSchema2WithCalibration(const param_model_t            *params,
                                                               const imu_bmi270_calibration_t *calibration)
{
    flash_param_image_v2_t image = {0};

    if (ParameterManagement_Validate(params) == 0U)
    {
        return FLASH_PARAM_STATUS_INVALID;
    }
    image.magic                             = FLASH_PARAM_MAGIC;
    image.schema_version                    = 2UL;
    image.sequence                          = 7UL;
    image.payload_size                      = sizeof(image.payload);
    image.payload.params.version            = 1UL;
    image.payload.params.max_linear_mps     = params->max_linear_mps;
    image.payload.params.max_angular_rps    = params->max_angular_rps;
    image.payload.params.speed_ramp_mps2    = params->speed_ramp_mps2;
    image.payload.params.angular_ramp_rps2  = params->angular_ramp_rps2;
    image.payload.params.wheel_radius_m     = params->wheel_radius_m;
    image.payload.params.track_width_m      = params->track_width_m;
    image.payload.params.pid_integral_limit = params->pid_integral_limit;
    memcpy(image.payload.params.pid_kp, params->pid_kp, sizeof(image.payload.params.pid_kp));
    memcpy(image.payload.params.pid_ki, params->pid_ki, sizeof(image.payload.params.pid_ki));
    memcpy(image.payload.params.pid_kd, params->pid_kd, sizeof(image.payload.params.pid_kd));
    memcpy(image.payload.params.motor_dir, params->motor_dir, sizeof(image.payload.params.motor_dir));
    memcpy(image.payload.params.encoder_dir, params->encoder_dir, sizeof(image.payload.params.encoder_dir));
    memcpy(image.payload.params.current_zero_raw,
           params->current_zero_raw,
           sizeof(image.payload.params.current_zero_raw));
    image.payload.params.current_zero_valid = params->current_zero_valid;
    memcpy(image.payload.params.imu_gyro_bias_dps,
           params->imu_gyro_bias_dps,
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
    image.crc32         = FlashParamCodec_Crc32((const uint8_t *)&image, offsetof(flash_param_image_v2_t, crc32));
    image.commit_marker = FLASH_PARAM_COMMIT_MARKER;
    memset(flash_param_host_slots[0], 0xFF, FLASH_PARAM_IMAGE_SIZE);
    memcpy(flash_param_host_slots[0], &image, sizeof(image));
    return FLASH_PARAM_STATUS_OK;
}

flash_param_status_t FlashParamHost_SeedSchema2(const param_model_t *params)
{
    return FlashParamHost_SeedSchema2WithCalibration(params, 0);
}

flash_param_status_t FlashParamHost_SeedSchema3(const param_model_t *params)
{
    flash_param_image_v3_t image = {0};

    if (ParameterManagement_Validate(params) == 0U)
    {
        return FLASH_PARAM_STATUS_INVALID;
    }
    image.magic                            = FLASH_PARAM_MAGIC;
    image.schema_version                   = 3UL;
    image.sequence                         = 8UL;
    image.payload_size                     = sizeof(image.payload);
    image.payload.params.version           = 2UL;
    image.payload.params.max_linear_mps    = params->max_linear_mps;
    image.payload.params.max_angular_rps   = params->max_angular_rps;
    image.payload.params.speed_ramp_mps2   = params->speed_ramp_mps2;
    image.payload.params.angular_ramp_rps2 = params->angular_ramp_rps2;
    image.payload.params.wheel_radius_m    = params->wheel_radius_m;
    image.payload.params.track_width_m     = params->track_width_m;
    memcpy(image.payload.params.pid_kp, params->pid_kp, sizeof(image.payload.params.pid_kp));
    memcpy(image.payload.params.pid_ki, params->pid_ki, sizeof(image.payload.params.pid_ki));
    memcpy(image.payload.params.pid_kd, params->pid_kd, sizeof(image.payload.params.pid_kd));
    image.payload.params.pid_integral_limit = params->pid_integral_limit;
    memcpy(image.payload.params.motor_dir, params->motor_dir, sizeof(image.payload.params.motor_dir));
    memcpy(image.payload.params.encoder_dir, params->encoder_dir, sizeof(image.payload.params.encoder_dir));
    memcpy(image.payload.params.current_zero_raw,
           params->current_zero_raw,
           sizeof(image.payload.params.current_zero_raw));
    image.payload.params.current_zero_valid = params->current_zero_valid;
    memcpy(image.payload.params.line_threshold_raw,
           params->line_threshold_raw,
           sizeof(image.payload.params.line_threshold_raw));
    image.payload.params.line_active_low             = params->line_active_low;
    image.payload.params.line_kp                     = params->line_kp;
    image.payload.params.line_kd                     = params->line_kd;
    image.payload.params.line_speed_mps              = params->line_speed_mps;
    image.payload.params.line_slowdown_gain          = params->line_slowdown_gain;
    image.payload.params.line_detect_debounce_frames = params->line_detect_debounce_frames;
    image.payload.params.line_lost_debounce_frames   = params->line_lost_debounce_frames;
    memcpy(image.payload.params.current_observe_a,
           params->current_observe_a,
           sizeof(image.payload.params.current_observe_a));
    memcpy(image.payload.params.current_soft_limit_a,
           params->current_soft_limit_a,
           sizeof(image.payload.params.current_soft_limit_a));
    memcpy(image.payload.params.current_fault_a, params->current_fault_a, sizeof(image.payload.params.current_fault_a));
    image.payload.params.current_fault_debounce_ms     = params->current_fault_debounce_ms;
    image.payload.params.straight_wheel_coupling_gain  = params->straight_wheel_coupling_gain;
    image.payload.params.straight_heading_kp           = params->straight_heading_kp;
    image.payload.params.straight_heading_hold_enabled = params->straight_heading_hold_enabled;
    ImuBmi270Calibration_Default(&image.payload.imu_calibration);
    image.crc32         = FlashParamCodec_Crc32((const uint8_t *)&image, offsetof(flash_param_image_v3_t, crc32));
    image.commit_marker = FLASH_PARAM_COMMIT_MARKER;
    memset(flash_param_host_slots[0], 0xFF, FLASH_PARAM_IMAGE_SIZE);
    memcpy(flash_param_host_slots[0], &image, sizeof(image));
    return FLASH_PARAM_STATUS_OK;
}
#else
static flash_param_status_t FlashParam_WatchdogEnterMaintenance(void)
{
    return (FlashStorage_WatchdogEnterMaintenance() != 0U) ? FLASH_PARAM_STATUS_OK : FLASH_PARAM_STATUS_WRITE_ERROR;
}

static flash_param_status_t FlashParam_WatchdogExitMaintenance(void)
{
    return (FlashStorage_WatchdogExitMaintenance() != 0U) ? FLASH_PARAM_STATUS_OK : FLASH_PARAM_STATUS_WRITE_ERROR;
}

static const uint8_t *FlashParam_SlotData(uint8_t slot)
{
    return FlashStorage_SlotData(slot);
}

static flash_param_status_t FlashParam_StorageUnlock(void)
{
    return (FlashStorage_Unlock() != 0U) ? FLASH_PARAM_STATUS_OK : FLASH_PARAM_STATUS_WRITE_ERROR;
}

static void FlashParam_StorageLock(void)
{
    FlashStorage_Lock();
}

static flash_param_status_t FlashParamSlots_EraseSlot(uint8_t slot)
{
    return (FlashStorage_EraseSlot(slot) != 0U) ? FLASH_PARAM_STATUS_OK : FLASH_PARAM_STATUS_WRITE_ERROR;
}

static flash_param_status_t FlashParam_ProgramWord(uint8_t slot, size_t offset, uint32_t word)
{
    return (FlashStorage_ProgramWord(slot, offset, word) != 0U) ? FLASH_PARAM_STATUS_OK
                                                                : FLASH_PARAM_STATUS_WRITE_ERROR;
}
#endif

static uint8_t FlashParam_SequenceNewer(uint32_t lhs, uint32_t rhs)
{
    return (((int32_t)(lhs - rhs)) > 0) ? 1U : 0U;
}

static int8_t FlashParam_SelectActiveSlot(uint32_t *sequence)
{
    flash_param_bundle_t bundle;
    uint32_t             slot_sequence[FLASH_PARAM_SLOT_COUNT] = {0UL, 0UL};
    uint8_t              valid[FLASH_PARAM_SLOT_COUNT]         = {0U, 0U};

    for (uint8_t slot = 0U; slot < FLASH_PARAM_SLOT_COUNT; ++slot)
    {
        if (FlashParam_DecodeBundle(FlashParam_SlotData(slot), FLASH_PARAM_IMAGE_SIZE, &bundle, &slot_sequence[slot])
            == FLASH_PARAM_STATUS_OK)
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

flash_param_status_t FlashParamSlots_LoadBundle(flash_param_bundle_t *bundle)
{
    int8_t               active;
    flash_param_status_t legacy_status;

    if (bundle == 0)
    {
        return FLASH_PARAM_STATUS_INVALID;
    }
    active = FlashParam_SelectActiveSlot(0);
    if (active >= 0)
    {
        return FlashParam_DecodeBundle(FlashParam_SlotData((uint8_t)active), FLASH_PARAM_IMAGE_SIZE, bundle, 0);
    }
    legacy_status = FlashParamCodec_DecodeLegacy(FlashParam_SlotData(1U), FLASH_PARAM_IMAGE_SIZE, bundle);
    return legacy_status;
}

flash_param_status_t FlashParamSlots_SaveBundle(const flash_param_bundle_t *bundle)
{
    uint8_t              image[FLASH_PARAM_IMAGE_SIZE];
    flash_param_bundle_t verify_bundle;
    uint32_t             active_sequence = 0UL;
    uint32_t             verify_sequence = 0UL;
    int8_t               active;
    uint8_t              target;
    flash_param_status_t status;
    flash_param_status_t watchdog_status;
    const size_t         commit_offset = offsetof(flash_param_image_t, commit_marker);

    if (FlashParamCodec_BundleValid(bundle) == 0U)
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
    status = FlashParamSlots_EraseSlot(target);
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
    status =
        FlashParam_DecodeBundle(FlashParam_SlotData(target), FLASH_PARAM_IMAGE_SIZE, &verify_bundle, &verify_sequence);
    if (status != FLASH_PARAM_STATUS_OK || verify_sequence != active_sequence + 1UL
        || memcmp(&verify_bundle, bundle, sizeof(*bundle)) != 0)
    {
        return FLASH_PARAM_STATUS_WRITE_ERROR;
    }
    return FLASH_PARAM_STATUS_OK;
}

flash_param_status_t FlashParamSlots_Erase(void)
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
    status = FlashParamSlots_EraseSlot(0U);
    if (status == FLASH_PARAM_STATUS_OK)
    {
        status = FlashParamSlots_EraseSlot(1U);
    }
    FlashParam_StorageLock();
    watchdog_status = FlashParam_WatchdogExitMaintenance();
    if (status == FLASH_PARAM_STATUS_OK && watchdog_status != FLASH_PARAM_STATUS_OK)
    {
        status = watchdog_status;
    }
    return status;
}
