#include "power_management_service.h"

#include "adc_monitor.h"
#include "chassis_layout.h"
#include "motor_driver.h"
#include "platform_critical.h"
#include "state_estimation_service.h"

static power_management_config_t power_config;
static power_management_status_t power_status;
static uint32_t                  power_generation;
static uint8_t                   power_initialized;

static void PowerManagement_Publish(void)
{
    power_management_status_t next;
    platform_critical_state_t critical;

    AdcMonitor_GetState(&next);
    critical     = PlatformCritical_Enter();
    power_status = next;
    power_generation++;
    PlatformCritical_Exit(critical);
}

uint8_t PowerManagement_Init(const power_management_config_t *config)
{
    power_management_status_t initial_status;
    platform_critical_state_t critical;

    if (config == 0 || config->current_zero_max_speed_mps < 0.0f || config->update_period_ms == 0UL)
    {
        return 0U;
    }
    AdcMonitor_SetUpdatePeriodMs((uint32_t)config->update_period_ms);
    AdcMonitor_Init();
    AdcMonitor_GetState(&initial_status);
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
    AdcMonitor_Update();
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
    for (uint8_t index = 0U; index < POWER_MEASUREMENT_MOTOR_COUNT; ++index)
    {
        if (motor.effective_pwm[index] != 0
            || (ChassisLayout_MotorEnabled((motor_id_t)index) != 0U
                && (wheel.speed_valid[index] == 0U || wheel.speed_mps[index] < -power_config.current_zero_max_speed_mps
                    || wheel.speed_mps[index] > power_config.current_zero_max_speed_mps)))
        {
            stationary = 0U;
            break;
        }
    }
    AdcMonitor_SetCurrentZeroStationary(stationary);
}

void PowerManagement_RequestCurrentZeroCalibration(void)
{
    AdcMonitor_RequestCurrentZeroCalibration();
    PowerManagement_Publish();
}

void PowerManagement_ApplyCurrentZeroCalibration(const uint16_t zero_raw[POWER_MEASUREMENT_MOTOR_COUNT])
{
    AdcMonitor_ApplyCurrentZeroCalibration(zero_raw);
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
