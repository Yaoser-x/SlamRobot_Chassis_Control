#include "power_management_service.h"

#include "power_adc_driver.h"
#include "motor_hardware_layout.h"
#include "motor_driver.h"
#include "platform_critical.h"
#include "state_estimation_service.h"

static power_management_config_t power_config;
static power_management_status_t power_status;
static uint32_t                  power_generation;
static uint8_t                   power_initialized;

static void PowerManagement_MapDriverState(const power_adc_driver_state_t *source, power_management_status_t *target)
{
    if (source == 0 || target == 0)
    {
        return;
    }
    *target = (power_management_status_t){0};
    for (uint8_t index = 0U; index < POWER_MANAGEMENT_MOTOR_COUNT; ++index)
    {
        target->raw_current[index]           = source->raw_current[index];
        target->current_zero_raw[index]      = source->current_zero_raw[index];
        target->current_a[index]             = source->current_a[index];
        target->current_mean_a[index]        = source->current_mean_a[index];
        target->current_rms_a[index]         = source->current_rms_a[index];
        target->current_peak_a[index]        = source->current_peak_a[index];
        target->current_signed_mean_a[index] = source->current_signed_mean_a[index];
        target->current_noise_a[index]       = source->current_noise_a[index];
        target->current_sample_count[index]  = source->current_sample_count[index];
        target->current_zero_span_raw[index] = source->current_zero_span_raw[index];
        target->current_quality_flags[index] = source->current_quality_flags[index];
    }
    target->raw_battery                = source->raw_battery;
    target->raw_left_current           = source->raw_left_current;
    target->raw_right_current          = source->raw_right_current;
    target->battery_voltage            = source->battery_voltage;
    target->left_current_a             = source->left_current_a;
    target->right_current_a            = source->right_current_a;
    target->current_zero_sample_count  = source->current_zero_sample_count;
    target->raw_sample_count           = source->raw_sample_count;
    target->missed_window_count        = source->missed_window_count;
    target->sample_rate_hz_milli       = source->sample_rate_hz_milli;
    target->valid_flags                = source->valid_flags;
    target->invalid_reason_flags       = source->invalid_reason_flags;
    target->samples_ready              = source->samples_ready;
    target->current_zero_valid         = source->current_zero_valid;
    target->current_valid              = source->current_valid;
    target->current_control_valid      = source->current_control_valid;
    target->current_control_valid_mask = source->current_control_valid_mask;
}

static void PowerManagement_Publish(void)
{
    power_adc_driver_state_t  raw;
    power_management_status_t next;
    platform_critical_state_t critical;

    PowerAdcDriver_GetState(&raw);
    PowerManagement_MapDriverState(&raw, &next);
    critical     = PlatformCritical_Enter();
    power_status = next;
    power_generation++;
    PlatformCritical_Exit(critical);
}

uint8_t PowerManagement_Init(const power_management_config_t *config)
{
    power_adc_driver_state_t  raw;
    power_management_status_t initial_status;
    platform_critical_state_t critical;

    if (config == 0 || config->current_zero_max_speed_mps < 0.0f || config->update_period_ms == 0UL)
    {
        return 0U;
    }
    PowerAdcDriver_SetUpdatePeriodMs((uint32_t)config->update_period_ms);
    PowerAdcDriver_Init();
    PowerAdcDriver_GetState(&raw);
    PowerManagement_MapDriverState(&raw, &initial_status);
    critical          = PlatformCritical_Enter();
    power_config      = *config;
    power_status      = initial_status;
    power_generation  = 0UL;
    power_initialized = 1U;
    PlatformCritical_Exit(critical);
    return 1U;
}

uint8_t PowerManagement_IsInitialized(void)
{
    return power_initialized;
}

void PowerManagement_Update(void)
{
    if (power_initialized == 0U)
    {
        return;
    }
    PowerAdcDriver_Update();
    PowerManagement_Publish();
}

void PowerManagement_UpdateStationary(void)
{
    state_estimation_wheel_status_t wheel;
    motor_driver_state_t            motor;
    uint8_t                         stationary = 1U;

    if (power_initialized == 0U)
    {
        return;
    }
    (void)StateEstimation_GetWheel(&wheel);
    MotorDriver_GetState(&motor);
    for (uint8_t index = 0U; index < POWER_MANAGEMENT_MOTOR_COUNT; ++index)
    {
        if (motor.effective_pwm[index] != 0
            || (MotorHardwareLayout_MotorEnabled((motor_id_t)index) != 0U
                && (wheel.speed_valid[index] == 0U || wheel.speed_mps[index] < -power_config.current_zero_max_speed_mps
                    || wheel.speed_mps[index] > power_config.current_zero_max_speed_mps)))
        {
            stationary = 0U;
            break;
        }
    }
    PowerAdcDriver_SetCurrentZeroStationary(stationary);
}

void PowerManagement_RequestCurrentZeroCalibration(void)
{
    PowerAdcDriver_RequestCurrentZeroCalibration();
    PowerManagement_Publish();
}

void PowerManagement_ApplyCurrentZeroCalibration(const uint16_t zero_raw[POWER_MANAGEMENT_MOTOR_COUNT])
{
    PowerAdcDriver_ApplyCurrentZeroCalibration(zero_raw);
    PowerManagement_Publish();
}

uint32_t PowerManagement_GetStatus(power_management_status_t *status)
{
    platform_critical_state_t critical;
    uint32_t                  generation;

    if (status == 0)
    {
        return 0UL;
    }
    critical   = PlatformCritical_Enter();
    *status    = power_status;
    generation = power_generation;
    PlatformCritical_Exit(critical);
    return generation;
}
