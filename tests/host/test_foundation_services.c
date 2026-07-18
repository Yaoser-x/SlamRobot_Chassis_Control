#include "robot_config.h"
#include "imu_calibration_orchestrator.h"
#include "power_adc_driver.h"
#include "motor_hardware_layout.h"
#include "wheel_encoder_driver.h"
#include "host_platform.h"
#include "bmi270_driver.h"
#include "motion_control_service.h"
#include "motion_control_maintenance.h"
#include "motor_driver.h"
#include "parameter_management_service.h"
#include "parameter_persistence_backend.h"
#include "power_management_service.h"
#include "state_estimation_service.h"
#include "state_estimation_maintenance.h"
#include "imu_estimation_pipeline.h"
#include "wheel_estimation_pipeline.h"
#include "system_monitoring_service.h"

#include <stdio.h>
#include <stdlib.h>

static wheel_encoder_sample_t          fake_wheel_raw;
static state_estimation_wheel_status_t fake_wheel;
static bmi270_driver_state_t           fake_imu_device;
static state_estimation_imu_status_t   fake_imu;
static bmi270_sample_t                 fake_imu_sample;
static uint8_t                         fake_imu_sample_pending;
static uint8_t                         fake_imu_update_enabled;
static uint8_t                         fake_imu_reentrant_probe;
static uint8_t                         fake_imu_reentrant_probe_result;
static power_adc_driver_state_t        fake_adc;
static motor_driver_state_t            fake_motor;
static float                           last_wheel_radius_m;
static imu_calibration_t               fake_calibration;
static uint8_t                         fake_zero_stationary;
static uint32_t                        external_call_in_critical_count;
static uint32_t                        persistence_save_count;
static flash_param_bundle_t            last_saved_bundle;
static flash_param_status_t            fake_persistence_save_status;

static void require_int(int condition, const char *message)
{
    if (condition == 0)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

static void record_external_call(void)
{
    if (HostPlatform_CriticalActive() != 0U)
    {
        external_call_in_critical_count++;
    }
}

void WheelEncoderDriver_Init(void)
{
    record_external_call();
    fake_wheel_raw                    = (wheel_encoder_sample_t){0};
    fake_wheel_raw.count[MOTOR_ID_M2] = 17U;
}

void WheelEncoderDriver_Read(wheel_encoder_sample_t *sample)
{
    record_external_call();
    *sample = fake_wheel_raw;
}

void WheelEstimationPipeline_Init(void)
{
    fake_wheel                    = (state_estimation_wheel_status_t){0};
    fake_wheel.count[MOTOR_ID_M2] = (int32_t)fake_wheel_raw.count[MOTOR_ID_M2];
}

void WheelEstimationPipeline_Update(const wheel_encoder_sample_t    *sample,
                                    uint32_t                         now_ms,
                                    float                            wheel_radius_m,
                                    const int8_t                     encoder_dir[STATE_ESTIMATION_MOTOR_COUNT],
                                    state_estimation_wheel_status_t *status)
{
    (void)encoder_dir;
    record_external_call();
    last_wheel_radius_m       = wheel_radius_m;
    fake_wheel.last_update_ms = now_ms;
    for (uint8_t index = 0U; index < STATE_ESTIMATION_MOTOR_COUNT; ++index)
    {
        fake_wheel.count[index] = (int32_t)sample->count[index];
    }
    *status = fake_wheel;
}

void Bmi270Driver_Init(void)
{
    record_external_call();
    fake_imu_device            = (bmi270_driver_state_t){0};
    fake_imu_device.chip_id    = 0x24U;
    fake_imu_device.init_state = BMI270_DRIVER_INIT_SAMPLING;
    fake_imu_sample            = (bmi270_sample_t){0};
    fake_imu_sample.accel_g[2] = 1.0f;
    fake_imu_sample_pending    = 0U;
    fake_imu_update_enabled    = 1U;
    fake_imu_reentrant_probe   = 0U;
}

uint8_t Bmi270Driver_Update(void)
{
    record_external_call();
    if (fake_imu_update_enabled == 0U)
    {
        return 0U;
    }
    if (fake_imu_reentrant_probe != 0U)
    {
        fake_imu_reentrant_probe_result = StateEstimation_ProbeImu();
    }
    fake_imu_device.last_update_ms += 10U;
    fake_imu_device.sample_count++;
    fake_imu_sample.timestamp_ms = fake_imu_device.last_update_ms;
    fake_imu_sample_pending      = 1U;
    return 1U;
}

uint8_t Bmi270Driver_TakeSample(bmi270_sample_t *sample)
{
    if (fake_imu_sample_pending == 0U)
    {
        return 0U;
    }
    *sample                 = fake_imu_sample;
    fake_imu_sample_pending = 0U;
    return 1U;
}

void Bmi270Driver_OnDataReadyFromIsr(void)
{
    record_external_call();
    fake_imu_device.drdy_count++;
}

void Bmi270Driver_GetState(bmi270_driver_state_t *state)
{
    record_external_call();
    *state = fake_imu_device;
}

uint8_t Bmi270Driver_SetEnabled(uint8_t enabled)
{
    fake_imu_device.enabled = enabled;
    fake_imu_device.online  = 0U;
    return 1U;
}

uint8_t Bmi270Driver_SetProfile(imu_bmi270_profile_id_t profile)
{
    fake_imu_device.profile = (uint8_t)profile;
    return 1U;
}

uint8_t Bmi270Driver_ProbeNow(void)
{
    return 1U;
}

uint8_t Bmi270Driver_ConfigNow(void)
{
    fake_imu_device.online = 1U;
    return 1U;
}

uint8_t Bmi270Driver_Diagnose(imu_bmi270_diag_t *diag)
{
    *diag = (imu_bmi270_diag_t){0};
    return 1U;
}

void ImuEstimationPipeline_ResetRuntime(uint32_t now_ms)
{
    (void)now_ms;
    fake_imu.last_update_ms     = 0UL;
    fake_imu.sensor_time        = 0UL;
    fake_imu.filter_initialized = 0U;
}

void ImuEstimationPipeline_ArmAutomaticCalibration(uint32_t now_ms, state_estimation_imu_status_t *status)
{
    record_external_call();
    ImuEstimationPipeline_ResetRuntime(now_ms);
    fake_imu.gyro_calibrated      = 0U;
    status->gyro_auto_cal_enabled = 1U;
    status->gyro_calibrated       = 0U;
    status->gyro_auto_cal_state =
        (status->enabled != 0U) ? STATE_ESTIMATION_IMU_AUTO_CAL_WAIT_ONLINE : STATE_ESTIMATION_IMU_AUTO_CAL_DISABLED;
    status->gyro_auto_cal_attempts    = 0U;
    status->gyro_auto_cal_last_result = 0U;
    status->gyro_cal_fail_reason      = 0U;
    status->gyro_cal_fail_axis        = 0U;
    status->gyro_cal_sample_count     = 0U;
    status->quaternion[0]             = 1.0f;
    status->roll_deg                  = 0.0f;
    status->pitch_deg                 = 0.0f;
    status->yaw_deg                   = 0.0f;
}

void ImuEstimationPipeline_Init(void)
{
    fake_imu                 = (state_estimation_imu_status_t){0};
    fake_imu.chip_id         = fake_imu_device.chip_id;
    fake_imu.init_state      = STATE_ESTIMATION_IMU_INIT_STATE_SAMPLING;
    fake_imu.body_accel_g[2] = 1.0f;
}

void ImuEstimationPipeline_Process(const bmi270_sample_t         *sample,
                                   const bmi270_driver_state_t   *device,
                                   state_estimation_imu_status_t *status)
{
    fake_imu.chip_id        = device->chip_id;
    fake_imu.last_update_ms = sample->timestamp_ms;
    fake_imu.sample_count   = device->sample_count;
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        fake_imu.body_accel_g[axis]       = sample->accel_g[axis];
        fake_imu.gyro_corrected_dps[axis] = sample->gyro_dps[axis];
        status->body_accel_g[axis]        = fake_imu.body_accel_g[axis];
        status->gyro_corrected_dps[axis]  = fake_imu.gyro_corrected_dps[axis];
    }
    status->chip_id            = fake_imu.chip_id;
    status->last_update_ms     = fake_imu.last_update_ms;
    status->sample_count       = fake_imu.sample_count;
    status->gyro_calibrated    = fake_imu.gyro_calibrated;
    status->filter_initialized = fake_imu.filter_initialized;
}

void ImuEstimationPipeline_ServiceCalibration(uint32_t                       now_ms,
                                              uint8_t                        stationary,
                                              state_estimation_imu_status_t *status)
{
    (void)now_ms;
    if (stationary != 0U)
    {
        status->gyro_calibrated  = 1U;
        fake_imu.gyro_calibrated = 1U;
    }
}

uint8_t ImuEstimationPipeline_ApplyCalibration(const imu_calibration_t *calibration)
{
    record_external_call();
    if (calibration == 0)
    {
        return 0U;
    }
    fake_calibration         = *calibration;
    fake_imu.gyro_calibrated = 1U;
    return 1U;
}

void ImuEstimationPipeline_GetCalibration(imu_calibration_t *calibration)
{
    record_external_call();
    *calibration = fake_calibration;
}

void ImuEstimationPipeline_ClearCalibration(void)
{
    record_external_call();
    fake_calibration = (imu_calibration_t){0};
}

uint8_t ImuEstimationPipeline_BeginCalibration(uint16_t samples, uint16_t interval_ms, uint8_t automatic)
{
    (void)samples;
    (void)interval_ms;
    (void)automatic;
    return 1U;
}

void PowerAdcDriver_Init(void)
{
    record_external_call();
    fake_adc                 = (power_adc_driver_state_t){0};
    fake_adc.battery_voltage = 11.8f;
}

void PowerAdcDriver_SetUpdatePeriodMs(uint32_t period_ms)
{
    require_int(period_ms == 20U, "ADC update period injected");
}

void PowerAdcDriver_Update(void)
{
    record_external_call();
    fake_adc.raw_sample_count++;
}

void PowerAdcDriver_GetState(power_adc_driver_state_t *state)
{
    record_external_call();
    *state = fake_adc;
}

void PowerAdcDriver_RequestCurrentZeroCalibration(void)
{
    record_external_call();
    fake_adc.current_zero_valid = 0U;
}

void PowerAdcDriver_ApplyCurrentZeroCalibration(const uint16_t zero_raw[MOTOR_ID_COUNT])
{
    record_external_call();
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        fake_adc.current_zero_raw[index] = zero_raw[index];
    }
    fake_adc.current_zero_valid = 1U;
}

void PowerAdcDriver_SetCurrentZeroStationary(uint8_t stationary)
{
    record_external_call();
    fake_zero_stationary = stationary;
}

void MotorDriver_GetState(motor_driver_state_t *state)
{
    record_external_call();
    *state = fake_motor;
}

uint8_t MotorHardwareLayout_MotorEnabled(motor_id_t motor)
{
    record_external_call();
    return (motor == MOTOR_ID_M2 || motor == MOTOR_ID_M3) ? 1U : 0U;
}

uint32_t MotionControl_GetStatus(motion_control_status_t *status)
{
    record_external_call();
    *status = (motion_control_status_t){0};
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        status->motor_effective_output_permille[index] = fake_motor.effective_pwm[index];
    }
    status->motor_enabled_mask = (uint8_t)((1U << MOTOR_ID_M2) | (1U << MOTOR_ID_M3));
    return status->generation;
}

motion_control_maintenance_result_t MotionControl_BeginMaintenance(void)
{
    record_external_call();
    return MOTION_CONTROL_MAINTENANCE_OK;
}

void MotionControl_EndMaintenance(void)
{
    record_external_call();
}

flash_param_status_t ParamPersistence_Save(const flash_param_bundle_t *bundle)
{
    record_external_call();
    last_saved_bundle = *bundle;
    persistence_save_count++;
    return fake_persistence_save_status;
}

flash_param_status_t ParamPersistence_Load(flash_param_bundle_t *bundle)
{
    (void)bundle;
    return FLASH_PARAM_STATUS_EMPTY;
}

static void test_parameter_owner_uses_injected_factory_and_monotonic_generation(void)
{
    const robot_config_t         *robot = RobotConfig_GetDefault();
    parameter_management_status_t status;
    param_model_t                 defaults;
    param_model_t                 ram;
    uint32_t                      generation;

    require_int(ParameterManagement_Init(&robot->parameter) != 0U, "parameter config accepted");
    generation = ParameterManagement_GetStatus(&status);
    require_int(generation == 1UL && status.initialized != 0U, "parameter init publishes generation one");
    require_int(status.params.track_width_m == 0.176f, "injected Beta2 track width preserved");

    ram                = status.params;
    ram.max_linear_mps = 0.42f;
    require_int(ParameterManagement_Set(&ram) != 0U, "RAM override accepted");
    require_int(ParameterManagement_GetStatus(&status) == generation + 1UL, "RAM override increments generation");
    require_int(status.params.max_linear_mps == 0.42f, "RAM override is current");
    ParameterManagement_Defaults(&defaults);
    require_int(defaults.max_linear_mps == robot->parameter.factory_defaults.max_linear_mps,
                "RAM override does not mutate factory defaults");
}

static void test_state_generations_and_freshness_are_independent(void)
{
    const robot_config_t           *robot = RobotConfig_GetDefault();
    state_estimation_status_t       status;
    state_estimation_wheel_status_t wheel;
    state_estimation_imu_status_t   imu;
    uint32_t                        wheel_generation;

    require_int(StateEstimation_Init(&robot->state) != 0U, "state config accepted");
    require_int(StateEstimation_GetWheel(&wheel) == 0UL && wheel.count[MOTOR_ID_M2] == 0,
                "wheel estimate stays empty until the first raw sample is mapped");
    require_int(StateEstimation_GetImu(&imu) == 0UL && imu.chip_id == 0U,
                "IMU estimate stays empty until the first raw sample is mapped");
    fake_wheel.speed_valid_all          = 1U;
    fake_wheel.speed_valid[MOTOR_ID_M2] = 1U;
    fake_wheel.speed_valid[MOTOR_ID_M3] = 1U;
    fake_wheel.speed_mps[MOTOR_ID_M2]   = 0.1f;
    fake_wheel.speed_mps[MOTOR_ID_M3]   = 0.1f;
    StateEstimation_UpdateWheel(100U);
    wheel_generation = StateEstimation_GetWheel(&wheel);
    require_int(wheel_generation == 1UL && wheel.last_update_ms == 100U, "wheel publish generation one");
    require_int(last_wheel_radius_m == 0.035f, "wheel conversion consumes parameter snapshot");

    fake_imu_device.last_update_ms = 90U;
    require_int(StateEstimation_RunImuCycle() != 0U, "IMU cycle updates");
    (void)StateEstimation_GetStatus(120U, &status);
    require_int(status.wheel_generation == 1UL && status.imu_sample_generation == 1UL
                    && status.imu_status_generation == 1UL && status.imu_generation == 2UL,
                "wheel, IMU sample, and IMU status generations publish independently");
    require_int(status.wheel_fresh != 0U && status.imu_fresh != 0U, "both chains initially fresh");
    fake_imu_update_enabled = 0U;
    require_int(StateEstimation_RunImuCycle() == 0U, "empty IMU cycle reports no sample");
    (void)StateEstimation_GetStatus(120U, &status);
    require_int(status.imu_sample_generation == 1UL, "empty IMU cycle does not publish a sample generation");
    fake_imu_update_enabled = 1U;
    (void)StateEstimation_GetStatus(151U, &status);
    require_int(status.wheel_fresh != 0U && status.imu_fresh == 0U,
                "IMU freshness expires independently of wheel freshness");
}

static void test_state_estimation_owns_imu_lifecycle(void)
{
    imu_calibration_t                 calibration;
    state_estimation_imu_status_t     status;
    state_estimation_imu_diagnostic_t diagnostic;
    uint32_t                          generation;

    require_int(StateEstimation_SetImuEnabled(0U) != 0U, "service disables IMU");
    (void)StateEstimation_GetImu(&status);
    require_int(status.enabled == 0U && status.online == 0U && status.last_update_ms == 0U,
                "disable clears online and freshness state");
    fake_imu_device.poll_fallback_count = 7U;
    require_int(StateEstimation_SetImuProfile(STATE_ESTIMATION_IMU_PROFILE_DEBUG) != 0U, "service changes IMU profile");
    (void)StateEstimation_GetImu(&status);
    require_int(status.enabled == 0U && status.poll_fallback_count == 7U,
                "profile change preserves disabled state and synchronizes fallback baseline");
    require_int(StateEstimation_RunImuCycle() != 0U, "profile reset resumes IMU sampling");
    (void)StateEstimation_GetImu(&status);
    require_int((status.quality_flags & STATE_ESTIMATION_IMU_QUALITY_POLL_FALLBACK) == 0U,
                "runtime reset synchronizes the persistent driver fallback counter");
    require_int(StateEstimation_ReinitializeImu() != 0U, "service reinitializes and enables IMU");
    (void)StateEstimation_GetImu(&status);
    require_int(status.enabled != 0U && status.online != 0U, "reinitialize publishes device state");
    require_int(StateEstimation_ProbeImu() != 0U, "service probes IMU");
    require_int(StateEstimation_DiagnoseImu(&diagnostic) != 0U, "service maps diagnostic DTO");

    fake_imu_reentrant_probe        = 1U;
    fake_imu_reentrant_probe_result = 1U;
    require_int(StateEstimation_RunImuCycle() != 0U, "IMU owner continues its active update");
    require_int(fake_imu_reentrant_probe_result == 0U,
                "concurrent lifecycle operation is rejected while update owns IMU");
    fake_imu_reentrant_probe = 0U;

    ParameterImuCalibration_Default(&calibration);
    calibration.gyro_bias_dps[0] = 9.0f;
    calibration.crc              = ParameterImuCalibration_Crc(&calibration);
    require_int(StateEstimation_ApplyImuCalibration(&calibration) != 0U,
                "service accepts the historical boot calibration");
    generation = StateEstimation_GetImu(&status);
    require_int(status.gyro_calibrated != 0U, "historical boot calibration is initially applied");
    require_int(StateEstimation_ArmAutomaticImuCalibration() == (uint8_t)STATE_ESTIMATION_RESULT_OK,
                "service rearms automatic calibration for every boot");
    require_int(StateEstimation_GetImu(&status) > generation && status.gyro_calibrated == 0U
                    && status.gyro_auto_cal_state == STATE_ESTIMATION_IMU_AUTO_CAL_WAIT_ONLINE,
                "rearm publishes an uncalibrated wait state and advances generation");
    require_int(StateEstimation_GetImuCalibration(&calibration) == (uint8_t)STATE_ESTIMATION_RESULT_OK
                    && calibration.gyro_bias_dps[0] == 9.0f,
                "service rearm preserves the historical calibration model");
}

static void test_power_owner_publishes_and_gates_zero_calibration(void)
{
    const robot_config_t     *robot = RobotConfig_GetDefault();
    power_management_status_t status;
    uint32_t                  generation;

    require_int(PowerManagement_Init(&robot->power) != 0U, "power config accepted");
    require_int(PowerManagement_GetStatus(&status) == 0UL && status.battery_voltage == 11.8f,
                "ADC initialization state is visible without a runtime generation");
    fake_wheel.speed_mps[MOTOR_ID_M2] = 0.0f;
    fake_wheel.speed_mps[MOTOR_ID_M3] = 0.0f;
    StateEstimation_UpdateWheel(130U);
    fake_adc.battery_voltage = 12.3f;
    PowerManagement_Update();
    generation = PowerManagement_GetStatus(&status);
    require_int(generation == 1UL && status.battery_voltage == 12.3f, "power update publishes complete facts");

    fake_motor = (motor_driver_state_t){0};
    PowerManagement_UpdateStationary();
    require_int(fake_zero_stationary != 0U, "valid stopped wheels allow zero calibration");
    fake_motor.effective_pwm[MOTOR_ID_M2] = 1;
    PowerManagement_UpdateStationary();
    require_int(fake_zero_stationary == 0U, "nonzero motor output blocks zero calibration");
}

static void test_system_owner_uses_injected_strict_timeout_boundary(void)
{
    const robot_config_t      *robot = RobotConfig_GetDefault();
    system_monitoring_status_t status;
    uint32_t                   generation;

    require_int(SystemMonitoring_Init(&robot->system, 0xA5UL) != 0U, "system config accepted");
    generation = SystemMonitoring_GetStatus(&status);
    require_int(status.reset_reason_flags == 0xA5UL, "reset reason owned by System");
    SystemMonitoring_Heartbeat(SYSTEM_MONITORING_TASK_HOST, 100U);
    SystemMonitoring_UpdateTimeouts(140U);
    (void)SystemMonitoring_GetStatus(&status);
    require_int(status.task_health.timed_out[SYSTEM_MONITORING_TASK_HOST] == 0U,
                "timeout does not fire at exact boundary");
    SystemMonitoring_UpdateTimeouts(141U);
    require_int(SystemMonitoring_GetStatus(&status) > generation, "system generation is monotonic");
    require_int(status.task_health.timed_out[SYSTEM_MONITORING_TASK_HOST] != 0U,
                "timeout fires one millisecond beyond boundary");
    require_int(status.task_health.timeout_count[SYSTEM_MONITORING_TASK_HOST] == 1UL,
                "one transition increments timeout count once");
}

static void run_stationary_auto_calibration(uint32_t start_ms)
{
    fake_motor               = (motor_driver_state_t){0};
    fake_imu.gyro_calibrated = 0U;
    fake_imu.body_accel_g[0] = 0.0f;
    fake_imu.body_accel_g[1] = 0.0f;
    fake_imu.body_accel_g[2] = 1.0f;
    for (uint32_t sample = 0U; sample < 110U; ++sample)
    {
        (void)StateEstimation_RunImuCycle();
        ImuCalibrationOrchestrator_ProcessSample(start_ms + sample * 10U);
    }
}

static void test_persistence_policy_is_not_a_dead_config_field(void)
{
    param_model_t runtime_params;

    persistence_save_count = 0UL;
    ImuCalibrationOrchestrator_Init(0U, 1U);
    run_stationary_auto_calibration(1000U);
    ImuCalibrationOrchestrator_ProcessPersistence(3000U);
    require_int(persistence_save_count == 0UL, "disabled IMU persistence does not write Flash");

    (void)ParameterManagement_GetSnapshot(&runtime_params);
    runtime_params.current_zero_raw[MOTOR_ID_M2] = 321U;
    runtime_params.current_zero_valid            = 1U;
    require_int(ParameterManagement_Set(&runtime_params) != 0U, "existing RAM current zero accepted");
    ImuCalibrationOrchestrator_Init(1U, 0U);
    run_stationary_auto_calibration(4000U);
    ImuCalibrationOrchestrator_ProcessPersistence(6000U);
    ImuCalibrationOrchestrator_ProcessPersistence(7000U);
    require_int(persistence_save_count == 1UL, "enabled IMU persistence writes exactly once");
    require_int(last_saved_bundle.params.current_zero_valid == 0U,
                "disabled current-zero persistence omits current calibration");
    require_int(last_saved_bundle.params.current_zero_raw[MOTOR_ID_M2] == 0U,
                "disabled current-zero persistence clears the fixed-schema payload field");
    (void)ParameterManagement_GetSnapshot(&runtime_params);
    require_int(runtime_params.current_zero_valid != 0U && runtime_params.current_zero_raw[MOTOR_ID_M2] == 321U,
                "omitting current-zero persistence does not erase the RAM override");

    persistence_save_count       = 0UL;
    fake_persistence_save_status = FLASH_PARAM_STATUS_WRITE_ERROR;
    ImuCalibrationOrchestrator_Init(1U, 0U);
    run_stationary_auto_calibration(7000U);
    ImuCalibrationOrchestrator_ProcessPersistence(9000U);
    ImuCalibrationOrchestrator_ProcessPersistence(10000U);
    ImuCalibrationOrchestrator_ProcessPersistence(11000U);
    ImuCalibrationOrchestrator_ProcessPersistence(12000U);
    require_int(persistence_save_count == 3UL, "failed IMU persistence stops after three bounded attempts");
    fake_persistence_save_status = FLASH_PARAM_STATUS_OK;
}

static void test_factory_reset_persists_before_publishing_defaults(void)
{
    param_model_t params;

    (void)ParameterManagement_GetSnapshot(&params);
    params.max_linear_mps = 0.4f;
    require_int(ParameterManagement_Set(&params) != 0U, "RAM override accepted before reset");
    persistence_save_count = 0UL;
    require_int(ParameterManagement_ResetAndSaveDefaults() != 0U, "factory reset bundle saved");
    require_int(persistence_save_count == 1UL && last_saved_bundle.params.max_linear_mps == 0.5f,
                "factory defaults are the persisted payload");
    (void)ParameterManagement_GetSnapshot(&params);
    require_int(params.max_linear_mps == 0.5f, "factory defaults publish only after save succeeds");
}

int main(void)
{
    test_parameter_owner_uses_injected_factory_and_monotonic_generation();
    test_state_generations_and_freshness_are_independent();
    test_state_estimation_owns_imu_lifecycle();
    test_power_owner_publishes_and_gates_zero_calibration();
    test_system_owner_uses_injected_strict_timeout_boundary();
    test_factory_reset_persists_before_publishing_defaults();
    test_persistence_policy_is_not_a_dead_config_field();
    require_int(external_call_in_critical_count == 0UL, "no BSP or Service call occurs inside a critical section");
    (void)puts("foundation service tests passed");
    return 0;
}
