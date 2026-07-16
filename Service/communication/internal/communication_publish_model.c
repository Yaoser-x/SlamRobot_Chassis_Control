#include "communication_publish_model_service.h"

#include "command_management_service.h"
#include "wireless_communication_service.h"
#include "line_following_service.h"
#include "motion_control_service.h"
#include "platform_critical.h"
#include "power_management_service.h"
#include "power_on_self_test_service.h"
#include "safety_management_service.h"
#include "state_estimation_service.h"
#include "system_monitoring_service.h"
#include "teleoperation_service.h"
#include "host_communication_service.h"

static communication_publish_model_t        publish_models[2];
static uint8_t                              active_model;
static uint32_t                             publish_generation;
static communication_publish_model_config_t publish_config;

static uint8_t CommunicationPublishModel_IsFresh(uint32_t now_ms, uint32_t timestamp_ms, uint32_t timeout_ms)
{
    return (timestamp_ms != 0U && (uint32_t)(now_ms - timestamp_ms) <= timeout_ms) ? 1U : 0U;
}

uint8_t CommunicationPublishModel_Init(const communication_publish_model_config_t *config)
{
    platform_critical_state_t state;

    if (config == 0 || config->host_timeout_ms == 0U || config->esp12f_timeout_ms == 0U
        || config->line_timeout_ms == 0U)
    {
        return 0U;
    }
    state              = PlatformCritical_Enter();
    publish_config     = *config;
    publish_models[0]  = (communication_publish_model_t){0};
    publish_models[1]  = (communication_publish_model_t){0};
    active_model       = 0U;
    publish_generation = 0U;
    PlatformCritical_Exit(state);
    return 1U;
}

void CommunicationPublishModel_Update(uint32_t now_ms)
{
    communication_publish_model_t     next = {0};
    motion_control_status_t           motion;
    state_estimation_status_t         state;
    power_management_status_t         power;
    safety_management_status_t        safety;
    system_monitoring_status_t        system;
    line_following_status_t           line;
    teleoperation_status_t            teleoperation;
    host_communication_state_t        upper;
    wireless_communication_state_t    esp;
    power_on_self_test_result_t       post;
    system_monitoring_module_health_t modules;
    platform_critical_state_t         critical;
    uint8_t                           inactive;

    (void)MotionControl_GetStatus(&motion);
    (void)StateEstimation_GetStatus(now_ms, &state);
    (void)PowerManagement_GetStatus(&power);
    (void)SafetyManagement_GetStatus(&safety);
    (void)SystemMonitoring_GetStatus(&system);
    (void)LineFollowing_GetStatus(&line);
    (void)Teleoperation_GetStatus(&teleoperation);
    PowerOnSelfTest_GetResult(&post);
    HostCommunication_GetState(&upper);
    WirelessCommunication_GetState(&esp);

    for (uint8_t index = 0U; index < COMMUNICATION_PUBLISH_TASK_COUNT; ++index)
    {
        next.task_health.last_heartbeat_ms[index] = system.task_health.last_heartbeat_ms[index];
        next.task_health.timeout_count[index]     = system.task_health.timeout_count[index];
        next.task_health.missed_count[index]      = system.task_health.missed_count[index];
        next.task_health.timed_out[index]         = system.task_health.timed_out[index];
    }
    next.post.done                    = post.done;
    next.post.drv_fault_mask          = post.drv_fault_mask;
    next.post.adc_current_valid       = post.adc_current_valid;
    next.post.imu_chip_id             = post.imu_chip_id;
    next.post.encoder_speed_valid_all = post.encoder_speed_valid_all;
    next.post.drv_status              = (uint8_t)post.drv_status;
    next.post.adc_status              = (uint8_t)post.adc_status;
    next.post.imu_status              = (uint8_t)post.imu_status;
    next.post.encoder_status          = (uint8_t)post.encoder_status;
    next.post.error_flags             = post.error_flags;

    next.timestamp_ms = now_ms;
    for (uint8_t index = 0U; index < COMMUNICATION_PUBLISH_MOTOR_COUNT; ++index)
    {
        next.chassis.motor_target_mps[index]      = motion.motor_target_mps[index];
        next.chassis.motor_actual_mps[index]      = motion.motor_actual_mps[index];
        next.chassis.motor_output_permille[index] = motion.motor_effective_output_permille[index];
        next.encoder.count[index]                 = state.wheel.count[index];
        next.encoder.speed_mps[index]             = state.wheel.speed_mps[index];
        next.encoder.speed_valid[index]           = state.wheel.speed_valid[index];
        next.safety.motor_current_a[index]        = safety.motor_current_a[index];
        if (state.wheel.anomaly_count[index] > 0U)
        {
            next.encoder.anomaly_mask |= (uint8_t)(1U << index);
        }
    }
    next.chassis.motor_enabled_mask   = motion.motor_enabled_mask;
    next.encoder.speed_valid_all      = state.wheel.speed_valid_all;
    next.safety.battery_voltage       = safety.battery_voltage;
    next.safety.error_flags           = safety.error_flags;
    next.safety.latched_error_flags   = safety.latched_error_flags;
    next.safety.task_timeout_mask     = safety.task_timeout_mask;
    next.safety.control_mode          = safety.control_mode;
    next.safety.current_control_valid = safety.current_control_valid;
    next.safety.tim_break_active      = ((safety.error_flags & SYSTEM_ERROR_TIM_BREAK) != 0U) ? 1U : 0U;
    next.safety.motor_fault_mask      = safety.motor_fault_mask;
    next.current.invalid_reason_flags = power.invalid_reason_flags;
    next.current.valid_flags          = power.valid_flags;
    next.current.current_valid        = power.current_valid;

    next.imu.online                   = state.imu.online;
    next.imu.chip_id                  = state.imu.chip_id;
    next.imu.calibrated               = state.imu.gyro_calibrated;
    next.imu.sensor_time_valid        = state.imu.sensor_time_valid;
    next.imu.last_error               = state.imu.last_error;
    next.imu.timestamp_ms             = state.imu.last_update_ms;
    next.imu.sensor_time              = state.imu.sensor_time;
    next.imu.sample_count             = state.imu.sample_count;
    next.imu.quality_flags            = state.imu.quality_flags;
    next.imu.roll_deg                 = state.imu.roll_deg;
    next.imu.pitch_deg                = state.imu.pitch_deg;
    next.imu.yaw_deg                  = state.imu.yaw_deg;
    next.imu.temperature_c            = state.imu.temperature_c;
    next.imu.calibration_state        = state.imu.gyro_auto_cal_state;
    next.imu.calibration_fail_reason  = state.imu.gyro_cal_fail_reason;
    next.imu.calibration_sample_count = state.imu.gyro_cal_sample_count;
    next.imu.quality_counters[0]      = state.imu.spi_error_count;
    next.imu.quality_counters[1]      = state.imu.init_failure_count;
    next.imu.quality_counters[2]      = state.imu.fifo_overflow_count;
    next.imu.quality_counters[3]      = state.imu.timestamp_error_count;
    next.imu.quality_counters[4]      = state.imu.gyro_saturation_count;
    next.imu.quality_counters[5]      = state.imu.accel_anomaly_count;
    next.imu.quality_counters[6]      = state.imu.attitude_invalid_count;
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        next.imu.accel_g[axis]  = state.imu.body_accel_g[axis];
        next.imu.gyro_dps[axis] = state.imu.body_gyro_dps[axis];
    }
    for (uint8_t index = 0U; index < 4U; ++index)
    {
        next.imu.quaternion[index] = state.imu.quaternion[index];
    }

    next.control.emergency_stop     = safety.emergency_stop;
    next.control.fault_stop         = safety.fault_stop;
    next.control.line_enabled       = line.globally_enabled;
    next.control.active_source      = (uint8_t)CommandManagement_GetActiveSource(now_ms);
    next.control.reset_reason_flags = system.reset_reason_flags;

    next.communication.upper.checksum_errors      = upper.rx_checksum_errors;
    next.communication.upper.timeout_resets       = upper.rx_timeout_resets;
    next.communication.upper.rx_overflows         = upper.rx_overwrite_count;
    next.communication.upper.tx_drops             = upper.tx_busy_drops;
    next.communication.upper.last_valid_frame_ms  = upper.last_valid_frame_ms;
    next.communication.upper_last_rx_timestamp_ms = HostCommunication_GetLastRxTimestamp();
    next.communication.esp12f.rx_frames           = esp.rx_frames;
    next.communication.esp12f.checksum_errors     = esp.rx_checksum_errors;
    next.communication.esp12f.length_errors       = esp.rx_length_errors;
    next.communication.esp12f.timeout_resets      = esp.rx_timeout_resets;
    next.communication.esp12f.rx_overflows        = esp.rx_overflow_errors;
    next.communication.esp12f.tx_drops            = esp.tx_busy_drops;
    next.communication.esp12f.last_valid_frame_ms = esp.last_rx_timestamp_ms;
    next.communication.esp12f.download_mode       = esp.boot_mode_download;

    next.modules.imu_online     = state.imu.online;
    next.modules.encoder_online = state.wheel.speed_valid_all;
    next.modules.motor_online   = ((safety.error_flags & SYSTEM_ERROR_DRV_FAULT) == 0U) ? 1U : 0U;
    next.modules.adc_online     = power.current_valid;
    next.modules.upper_online   = CommunicationPublishModel_IsFresh(now_ms,
                                                                  next.communication.upper_last_rx_timestamp_ms,
                                                                  publish_config.host_timeout_ms);
    next.modules.esp12f_online =
        (esp.boot_mode_download == 0U)
            ? CommunicationPublishModel_IsFresh(now_ms, esp.last_rx_timestamp_ms, publish_config.esp12f_timeout_ms)
            : 0U;
    next.modules.line_online =
        (line.sensor_valid != 0U)
            ? CommunicationPublishModel_IsFresh(now_ms, line.sensor_timestamp_ms, publish_config.line_timeout_ms)
            : 0U;
    next.modules.ps2_online = teleoperation.online;

    modules = (system_monitoring_module_health_t){
        .imu_online     = next.modules.imu_online,
        .encoder_online = next.modules.encoder_online,
        .motor_online   = next.modules.motor_online,
        .adc_online     = next.modules.adc_online,
        .host_online    = next.modules.upper_online,
        .esp12f_online  = next.modules.esp12f_online,
        .line_online    = next.modules.line_online,
        .ps2_online     = next.modules.ps2_online,
    };
    SystemMonitoring_SetModuleHealth(&modules);

    inactive                 = (uint8_t)(active_model ^ 1U);
    publish_models[inactive] = next;
    critical                 = PlatformCritical_Enter();
    publish_generation++;
    publish_models[inactive].generation = publish_generation;
    active_model                        = inactive;
    PlatformCritical_Exit(critical);
}

uint32_t CommunicationPublishModel_Get(communication_publish_model_t *out)
{
    platform_critical_state_t state;
    uint32_t                  generation;

    if (out == 0)
    {
        return 0U;
    }
    state      = PlatformCritical_Enter();
    *out       = publish_models[active_model];
    generation = out->generation;
    PlatformCritical_Exit(state);
    return generation;
}
