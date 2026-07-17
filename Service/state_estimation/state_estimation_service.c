#include "state_estimation_service.h"

#include "wheel_encoder_driver.h"
#include "bmi270_driver.h"
#include "imu_estimation_pipeline.h"
#include "imu_calibration_coordinator.h"
#include "imu_quality_monitor.h"
#include "parameter_management_service.h"
#include "platform_critical.h"
#include "wheel_estimation_pipeline.h"

static state_estimation_config_t       state_config;
static state_estimation_wheel_status_t wheel_status;
static state_estimation_imu_status_t   imu_status;
static uint32_t                        wheel_generation;
static uint32_t                        imu_generation;
static uint8_t                         state_initialized;

static void StateEstimation_ResetImuRuntime(uint8_t enabled)
{
    state_estimation_imu_status_t next = {0};
    bmi270_driver_state_t         device;
    platform_critical_state_t     critical;

    ImuEstimationPipeline_ResetRuntime();
    Bmi270Driver_GetState(&device);
    critical                   = PlatformCritical_Enter();
    next.quality_latched_flags = imu_status.quality_latched_flags;
    next.poll_fallback_count   = device.poll_fallback_count;
    next.gyro_calibrated       = imu_status.gyro_calibrated;
    next.gyro_auto_cal_enabled = imu_status.gyro_auto_cal_enabled;
    next.enabled               = enabled;
    next.init_state =
        (enabled != 0U) ? STATE_ESTIMATION_IMU_INIT_STATE_RESET : STATE_ESTIMATION_IMU_INIT_STATE_DISABLED;
    next.quaternion[0] = 1.0f;
    imu_status         = next;
    imu_generation++;
    PlatformCritical_Exit(critical);
}

static uint8_t StateEstimation_IsFresh(uint32_t now_ms, uint32_t timestamp_ms, uint32_t timeout_ms)
{
    return (timestamp_ms != 0UL && (uint32_t)(now_ms - timestamp_ms) <= timeout_ms) ? 1U : 0U;
}

static void StateEstimation_PublishImu(void)
{
    bmi270_driver_state_t         device;
    bmi270_sample_t               sample;
    state_estimation_imu_status_t next = imu_status;
    platform_critical_state_t     critical;

    Bmi270Driver_GetState(&device);
    ImuQualityMonitor_BeginCycle(&device, &next);
    while (Bmi270Driver_TakeSample(&sample) != 0U)
    {
        ImuEstimationPipeline_Process(&sample, &device, &next);
    }
    ImuQualityMonitor_EndCycle(&next);
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
    WheelEncoderDriver_Init();
    WheelEstimationPipeline_Init();
    Bmi270Driver_Init();
    ImuEstimationPipeline_Init();
    initial_wheel                     = (state_estimation_wheel_status_t){0};
    initial_imu                       = (state_estimation_imu_status_t){0};
    initial_imu.quaternion[0]         = 1.0f;
    initial_imu.gyro_auto_cal_enabled = 1U;
    initial_imu.gyro_auto_cal_state   = STATE_ESTIMATION_IMU_AUTO_CAL_WAIT_ONLINE;
    critical                          = PlatformCritical_Enter();
    state_config                      = *config;
    wheel_status                      = initial_wheel;
    imu_status                        = initial_imu;
    wheel_generation                  = 0UL;
    imu_generation                    = 0UL;
    state_initialized                 = 1U;
    PlatformCritical_Exit(critical);
    return 1U;
}

uint8_t StateEstimation_IsInitialized(void)
{
    return state_initialized;
}

uint8_t StateEstimation_SetImuEnabled(uint8_t enabled)
{
    uint8_t normalized = (enabled != 0U) ? 1U : 0U;
    uint8_t accepted   = Bmi270Driver_SetEnabled(normalized);

    if (accepted != 0U)
    {
        StateEstimation_ResetImuRuntime(normalized);
    }
    return accepted;
}

uint8_t StateEstimation_ProbeImu(void)
{
    uint8_t result = Bmi270Driver_ProbeNow();
    StateEstimation_PublishImu();
    return result;
}

uint8_t StateEstimation_ReinitializeImu(void)
{
    if (Bmi270Driver_SetEnabled(1U) == 0U)
    {
        return 0U;
    }
    StateEstimation_ResetImuRuntime(1U);
    uint8_t result = Bmi270Driver_ConfigNow();
    StateEstimation_PublishImu();
    return result;
}

uint8_t StateEstimation_SetImuProfile(state_estimation_imu_profile_t profile)
{
    state_estimation_imu_status_t status;
    uint8_t                       result;

    if ((uint8_t)profile > (uint8_t)STATE_ESTIMATION_IMU_PROFILE_DEBUG)
    {
        return 0U;
    }
    (void)StateEstimation_GetImu(&status);
    result = Bmi270Driver_SetProfile((imu_bmi270_profile_id_t)profile);
    if (result != 0U)
    {
        StateEstimation_ResetImuRuntime(status.enabled);
    }
    return result;
}

uint8_t StateEstimation_DiagnoseImu(state_estimation_imu_diagnostic_t *diagnostic)
{
    imu_bmi270_diag_t bsp_diagnostic;

    if (diagnostic == 0 || Bmi270Driver_Diagnose(&bsp_diagnostic) == 0U)
    {
        return 0U;
    }
    for (uint8_t attempt = 0U; attempt < 2U; ++attempt)
    {
        diagnostic->hal_status[attempt] = bsp_diagnostic.hal_status[attempt];
        for (uint8_t byte = 0U; byte < 3U; ++byte)
        {
            diagnostic->hal_rx[attempt][byte] = bsp_diagnostic.hal_rx[attempt][byte];
        }
    }
    for (uint8_t byte = 0U; byte < 3U; ++byte)
    {
        diagnostic->bitbang_rx[byte] = bsp_diagnostic.bitbang_rx[byte];
    }
    diagnostic->miso_nopull   = bsp_diagnostic.miso_nopull;
    diagnostic->miso_pullup   = bsp_diagnostic.miso_pullup;
    diagnostic->miso_pulldown = bsp_diagnostic.miso_pulldown;
    return 1U;
}

void StateEstimation_UpdateWheel(uint32_t now_ms)
{
    param_model_t                   params;
    wheel_encoder_sample_t          raw;
    state_estimation_wheel_status_t next;
    platform_critical_state_t       critical;

    if (state_initialized == 0U || ParameterManagement_GetSnapshot(&params) == 0UL)
    {
        return;
    }
    next = wheel_status;
    WheelEncoderDriver_Read(&raw);
    WheelEstimationPipeline_Update(&raw, now_ms, params.wheel_radius_m, params.encoder_dir, &next);
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
    platform_critical_state_t     critical;
    state_estimation_imu_status_t next;

    critical = PlatformCritical_Enter();
    next     = imu_status;
    PlatformCritical_Exit(critical);
    ImuEstimationPipeline_ServiceCalibration(now_ms, stationary, &next);
    critical   = PlatformCritical_Enter();
    imu_status = next;
    imu_generation++;
    PlatformCritical_Exit(critical);
}

uint8_t StateEstimation_ApplyImuCalibration(const imu_calibration_t *calibration)
{
    platform_critical_state_t critical;

    if (ImuEstimationPipeline_ApplyCalibration(calibration) == 0U)
    {
        return 0U;
    }
    critical                       = PlatformCritical_Enter();
    imu_status.gyro_calibrated     = 1U;
    imu_status.gyro_auto_cal_state = STATE_ESTIMATION_IMU_AUTO_CAL_SUCCESS;
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        imu_status.gyro_bias_dps[axis] = calibration->gyro_bias_dps[axis];
    }
    imu_generation++;
    PlatformCritical_Exit(critical);
    return 1U;
}

void StateEstimation_ClearImuCalibration(void)
{
    platform_critical_state_t critical;

    ImuEstimationPipeline_ClearCalibration();
    critical                   = PlatformCritical_Enter();
    imu_status.gyro_calibrated = 0U;
    imu_status.gyro_auto_cal_state =
        (imu_status.enabled != 0U) ? STATE_ESTIMATION_IMU_AUTO_CAL_WAIT_ONLINE : STATE_ESTIMATION_IMU_AUTO_CAL_DISABLED;
    imu_generation++;
    PlatformCritical_Exit(critical);
}

uint8_t StateEstimation_BeginImuCalibration(uint16_t samples, uint16_t interval_ms)
{
    state_estimation_imu_status_t status;

    (void)StateEstimation_GetImu(&status);
    if (status.enabled == 0U || status.online == 0U)
    {
        return 0U;
    }
    if (samples == 0U)
    {
        samples = 500U;
    }
    if (interval_ms == 0U)
    {
        interval_ms = 10U;
    }
    return ImuEstimationPipeline_BeginCalibration(samples, interval_ms, 0U);
}

void StateEstimation_GetImuCalibration(imu_calibration_t *calibration)
{
    ImuEstimationPipeline_GetCalibration(calibration);
}

void StateEstimation_InitCalibrationCoordinator(const state_estimation_calibration_ports_t *ports,
                                                uint8_t                                     first_save_needed,
                                                uint8_t                                     persist_imu_calibration,
                                                uint8_t                                     persist_current_zero)
{
    ImuCalibrationCoordinator_Init(ports, first_save_needed, persist_imu_calibration, persist_current_zero);
}

void StateEstimation_ServiceCalibrationCoordinator(uint32_t now_ms)
{
    ImuCalibrationCoordinator_ProcessSample(now_ms);
}

void StateEstimation_ServiceCalibrationPersistence(uint32_t now_ms)
{
    ImuCalibrationCoordinator_ProcessPersistence(now_ms);
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
