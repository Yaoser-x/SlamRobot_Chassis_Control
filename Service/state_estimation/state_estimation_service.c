#include "state_estimation_service.h"

#include "wheel_encoder_driver.h"
#include "bmi270_driver.h"
#include "parameter_management_service.h"
#include "platform_critical.h"

#include <string.h>

_Static_assert(sizeof(state_estimation_wheel_status_t) == sizeof(wheel_encoder_state_t),
               "State Estimation and encoder snapshots must remain layout-compatible");
_Static_assert(sizeof(state_estimation_imu_status_t) == sizeof(bmi270_driver_state_t),
               "State Estimation and BMI270 snapshots must remain layout-compatible");
_Static_assert(sizeof(imu_bmi270_calibration_t) == sizeof(bmi270_calibration_t),
               "Service and BMI270 calibration records must remain layout-compatible");

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
    bmi270_driver_state_t         raw;
    state_estimation_imu_status_t next;
    platform_critical_state_t     critical;

    Bmi270Driver_GetState(&raw);
    memcpy(&next, &raw, sizeof(next));
    critical   = PlatformCritical_Enter();
    imu_status = next;
    imu_generation++;
    PlatformCritical_Exit(critical);
}

uint8_t StateEstimation_Init(const state_estimation_config_t *config)
{
    state_estimation_wheel_status_t initial_wheel;
    state_estimation_imu_status_t   initial_imu;
    wheel_encoder_state_t           raw_wheel;
    bmi270_driver_state_t           raw_imu;
    platform_critical_state_t       critical;

    if (config == 0 || config->wheel_feedback_timeout_ms == 0UL || config->imu_fresh_timeout_ms == 0UL)
    {
        return 0U;
    }
    WheelEncoderDriver_Init();
    Bmi270Driver_Init();
    WheelEncoderDriver_GetState(&raw_wheel);
    Bmi270Driver_GetState(&raw_imu);
    memcpy(&initial_wheel, &raw_wheel, sizeof(initial_wheel));
    memcpy(&initial_imu, &raw_imu, sizeof(initial_imu));
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
    wheel_encoder_driver_config_t   driver_config;
    wheel_encoder_state_t           raw;
    state_estimation_wheel_status_t next;
    platform_critical_state_t       critical;

    if (state_initialized == 0U || ParameterManagement_GetSnapshot(&params) == 0UL)
    {
        return;
    }
    driver_config.wheel_radius_m = params.wheel_radius_m;
    for (uint8_t index = 0U; index < STATE_ESTIMATION_MOTOR_COUNT; ++index)
    {
        driver_config.encoder_dir[index] = params.encoder_dir[index];
    }
    WheelEncoderDriver_Update(now_ms, &driver_config);
    WheelEncoderDriver_GetState(&raw);
    memcpy(&next, &raw, sizeof(next));
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
    updated = Bmi270Driver_Update();
    StateEstimation_PublishImu();
    return updated;
}

void StateEstimation_OnImuDataReadyFromIsr(void)
{
    Bmi270Driver_OnDataReadyFromIsr();
}

void StateEstimation_ServiceImuCalibration(uint32_t now_ms, uint8_t stationary)
{
    Bmi270Driver_ServiceCalibration(now_ms, stationary);
    StateEstimation_PublishImu();
}

uint8_t StateEstimation_ApplyImuCalibration(const imu_bmi270_calibration_t *calibration)
{
    bmi270_calibration_t device_calibration;
    uint8_t              applied;

    if (calibration == 0)
    {
        return 0U;
    }
    memcpy(&device_calibration, calibration, sizeof(device_calibration));
    applied = Bmi270Driver_ApplyCalibration(&device_calibration);

    StateEstimation_PublishImu();
    return applied;
}

void StateEstimation_ClearImuCalibration(void)
{
    Bmi270Driver_ClearCalibration();
}

void StateEstimation_GetImuCalibration(imu_bmi270_calibration_t *calibration)
{
    bmi270_calibration_t device_calibration;

    if (calibration == 0)
    {
        return;
    }
    Bmi270Driver_GetCalibration(&device_calibration);
    memcpy(calibration, &device_calibration, sizeof(*calibration));
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
