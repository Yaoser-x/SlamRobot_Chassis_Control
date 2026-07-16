#include "state_estimation_service.h"

#include "encoder_driver.h"
#include "imu_bmi270.h"
#include "parameter_management_service.h"
#include "platform_critical.h"

static state_estimation_config_t       state_config;
static state_estimation_wheel_status_t wheel_status;
static state_estimation_imu_status_t   imu_status;
static uint32_t                        wheel_generation;
static uint32_t                        imu_generation;
static uint8_t                         state_initialized;

static uint8_t StateEstimation_IsFresh(uint32_t now_ms, uint32_t timestamp_ms, uint32_t timeout_ms)
{
    return (timestamp_ms != 0UL && (uint32_t)(now_ms - timestamp_ms) <= timeout_ms) ? 1U : 0U;
}

static void StateEstimation_PublishImu(void)
{
    state_estimation_imu_status_t next;
    platform_critical_state_t     critical;

    ImuBmi270_GetState(&next);
    critical   = PlatformCritical_Enter();
    imu_status = next;
    imu_generation++;
    PlatformCritical_Exit(critical);
}

uint8_t StateEstimation_Init(const state_estimation_config_t *config)
{
    state_estimation_wheel_status_t initial_wheel;
    state_estimation_imu_status_t   initial_imu;
    platform_critical_state_t       critical;

    if (config == 0 || config->wheel_feedback_timeout_ms == 0UL || config->imu_fresh_timeout_ms == 0UL)
    {
        return 0U;
    }
    EncoderDriver_Init();
    ImuBmi270_Init();
    EncoderDriver_GetState(&initial_wheel);
    ImuBmi270_GetState(&initial_imu);
    critical          = PlatformCritical_Enter();
    state_config      = *config;
    wheel_status      = initial_wheel;
    imu_status        = initial_imu;
    wheel_generation  = 0UL;
    imu_generation    = 0UL;
    state_initialized = 1U;
    PlatformCritical_Exit(critical);
    return 1U;
}

uint8_t StateEstimation_IsInitialized(void)
{
    return state_initialized;
}

void StateEstimation_UpdateWheel(uint32_t now_ms)
{
    param_model_t                   params;
    encoder_driver_config_t         driver_config;
    state_estimation_wheel_status_t next;
    platform_critical_state_t       critical;

    if (state_initialized == 0U || ParameterManagement_GetSnapshot(&params) == 0UL)
    {
        return;
    }
    driver_config.wheel_radius_m = params.wheel_radius_m;
    for (uint8_t index = 0U; index < WHEEL_ESTIMATION_MOTOR_COUNT; ++index)
    {
        driver_config.encoder_dir[index] = params.encoder_dir[index];
    }
    EncoderDriver_Update(now_ms, &driver_config);
    EncoderDriver_GetState(&next);
    critical     = PlatformCritical_Enter();
    wheel_status = next;
    wheel_generation++;
    PlatformCritical_Exit(critical);
}

uint8_t StateEstimation_RunImuCycle(void)
{
    uint8_t updated;

    if (state_initialized == 0U)
    {
        return 0U;
    }
    updated = ImuBmi270_Update();
    StateEstimation_PublishImu();
    return updated;
}

void StateEstimation_OnImuDataReadyFromIsr(void)
{
    ImuBmi270_OnDataReadyFromIsr();
}

void StateEstimation_ServiceImuCalibration(uint32_t now_ms, uint8_t stationary)
{
    ImuBmi270_ServiceCalibration(now_ms, stationary);
    StateEstimation_PublishImu();
}

uint8_t StateEstimation_ApplyImuCalibration(const imu_bmi270_calibration_t *calibration)
{
    uint8_t applied = ImuBmi270_ApplyCalibration(calibration);

    StateEstimation_PublishImu();
    return applied;
}

void StateEstimation_ClearImuCalibration(void)
{
    ImuBmi270_ClearCalibration();
}

void StateEstimation_GetImuCalibration(imu_bmi270_calibration_t *calibration)
{
    ImuBmi270_GetCalibration(calibration);
}

uint32_t StateEstimation_GetWheel(state_estimation_wheel_status_t *status)
{
    platform_critical_state_t critical;
    uint32_t                  generation;

    if (status == 0)
    {
        return 0UL;
    }
    critical   = PlatformCritical_Enter();
    *status    = wheel_status;
    generation = wheel_generation;
    PlatformCritical_Exit(critical);
    return generation;
}

uint32_t StateEstimation_GetImu(state_estimation_imu_status_t *status)
{
    platform_critical_state_t critical;
    uint32_t                  generation;

    if (status == 0)
    {
        return 0UL;
    }
    critical   = PlatformCritical_Enter();
    *status    = imu_status;
    generation = imu_generation;
    PlatformCritical_Exit(critical);
    return generation;
}

uint32_t StateEstimation_GetStatus(uint32_t now_ms, state_estimation_status_t *status)
{
    platform_critical_state_t critical;

    if (status == 0)
    {
        return 0UL;
    }
    critical                 = PlatformCritical_Enter();
    status->wheel            = wheel_status;
    status->imu              = imu_status;
    status->wheel_generation = wheel_generation;
    status->imu_generation   = imu_generation;
    PlatformCritical_Exit(critical);
    status->wheel_fresh =
        StateEstimation_IsFresh(now_ms, status->wheel.last_update_ms, state_config.wheel_feedback_timeout_ms);
    status->imu_fresh = StateEstimation_IsFresh(now_ms, status->imu.last_update_ms, state_config.imu_fresh_timeout_ms);
    return status->wheel_generation + status->imu_generation;
}
