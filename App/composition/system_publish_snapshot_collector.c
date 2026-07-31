#include "system_publish_snapshot_collector.h"

#include "command_management_service.h"
#include "control_mode_coordinator.h"
#include "host_communication_service.h"
#include "line_following_service.h"
#include "motion_control_service.h"
#include "parameter_management_service.h"
#include "power_management_service.h"
#include "power_on_self_test_service.h"
#include "safety_management_service.h"
#include "state_estimation_service.h"
#include "system_monitoring_service.h"
#include "teleoperation_service.h"
#include "wireless_communication_service.h"

static uint8_t AppSystemPublishSnapshot_IsFresh(uint32_t now_ms, uint32_t timestamp_ms, uint32_t timeout_ms)
{
    return (timestamp_ms != 0U && (uint32_t)(now_ms - timestamp_ms) <= timeout_ms) ? 1U : 0U;
}

static uint8_t AppSystemPublishSnapshot_MapImuCalibrationState(uint8_t calibration_state)
{
    switch (calibration_state)
    {
        case STATE_ESTIMATION_IMU_AUTO_CAL_DISABLED:
            return SYSTEM_IMU_CAL_DISABLED;
        case STATE_ESTIMATION_IMU_AUTO_CAL_WAIT_ONLINE:
        case STATE_ESTIMATION_IMU_AUTO_CAL_WAIT_STATIONARY:
            return SYSTEM_IMU_CAL_WAIT;
        case STATE_ESTIMATION_IMU_AUTO_CAL_COLLECTING:
            return SYSTEM_IMU_CAL_RUNNING;
        case STATE_ESTIMATION_IMU_AUTO_CAL_SUCCESS:
            return SYSTEM_IMU_CAL_DONE;
        case STATE_ESTIMATION_IMU_AUTO_CAL_RETRY_WAIT:
            return SYSTEM_IMU_CAL_RETRY_WAIT;
        case STATE_ESTIMATION_IMU_AUTO_CAL_FAILED:
        default:
            return SYSTEM_IMU_CAL_FAILED;
    }
}

void AppSystemPublishSnapshot_Collect(uint32_t                                    now_ms,
                                      const communication_publish_model_config_t *config,
                                      communication_publish_model_t              *snapshot)
{
    motion_control_status_t        motion;
    state_estimation_status_t      state;
    power_management_status_t      power;
    safety_management_status_t     safety;
    command_management_status_t    command;
    system_monitoring_status_t     system;
    line_following_status_t        line;
    teleoperation_status_t         teleoperation;
    host_communication_state_t     upper;
    wireless_communication_state_t esp;
    power_on_self_test_result_t    post;
    control_mode_snapshot_t        control_mode;

    if (config == 0 || snapshot == 0)
    {
        return;
    }
    *snapshot = (communication_publish_model_t){0};
    (void)MotionControl_GetStatus(&motion);
    (void)StateEstimation_GetStatus(now_ms, &state);
    (void)PowerManagement_GetStatus(&power);
    (void)SafetyManagement_GetStatus(&safety);
    (void)CommandManagement_GetStatus(now_ms, &command);
    (void)SystemMonitoring_GetStatus(&system);
    (void)LineFollowing_GetStatus(&line);
    (void)Teleoperation_GetStatus(&teleoperation);
    (void)ControlModeCoordinator_GetSnapshot(&control_mode);
    PowerOnSelfTest_GetResult(&post);
    HostCommunication_GetState(&upper);
    WirelessCommunication_GetState(&esp);

    for (uint8_t index = 0U; index < COMMUNICATION_PUBLISH_TASK_COUNT; ++index)
    {
        snapshot->task_health.last_heartbeat_ms[index] = system.task_health.last_heartbeat_ms[index];
        snapshot->task_health.timeout_count[index]     = system.task_health.timeout_count[index];
        snapshot->task_health.missed_count[index]      = system.task_health.missed_count[index];
        snapshot->task_health.timed_out[index]         = system.task_health.timed_out[index];
    }
    snapshot->post.done                    = post.done;
    snapshot->post.drv_fault_mask          = post.drv_fault_mask;
    snapshot->post.adc_current_valid       = post.adc_current_valid;
    snapshot->post.imu_chip_id             = post.imu_chip_id;
    snapshot->post.encoder_speed_valid_all = post.encoder_speed_valid_all;
    snapshot->post.drv_status              = (uint8_t)post.drv_status;
    snapshot->post.adc_status              = (uint8_t)post.adc_status;
    snapshot->post.imu_status              = (uint8_t)post.imu_status;
    snapshot->post.encoder_status          = (uint8_t)post.encoder_status;
    snapshot->post.error_flags             = post.error_flags;
    snapshot->timestamp_ms                 = now_ms;
    snapshot->parameter_identity_crc32     = ParameterManagement_GetIdentityCrc32();

    for (uint8_t index = 0U; index < COMMUNICATION_PUBLISH_MOTOR_COUNT; ++index)
    {
        snapshot->chassis.motor_target_mps[index]      = motion.motor_target_mps[index];
        snapshot->chassis.motor_actual_mps[index]      = motion.motor_actual_mps[index];
        snapshot->chassis.motor_output_permille[index] = motion.motor_effective_output_permille[index];
        snapshot->encoder.count[index]                 = state.wheel.count[index];
        snapshot->encoder.speed_mps[index]             = state.wheel.speed_mps[index];
        snapshot->encoder.speed_valid[index]           = state.wheel.speed_valid[index];
        snapshot->safety.motor_current_a[index]        = safety.motor_current_a[index];
    }
    snapshot->encoder.anomaly_mask         = state.wheel.current_anomaly_mask | state.wheel.latched_for_host_mask;
    snapshot->encoder.anomaly_generation   = state.wheel.anomaly_delivery_generation;
    snapshot->chassis.motor_enabled_mask   = motion.motor_enabled_mask;
    snapshot->encoder.speed_valid_all      = state.wheel.speed_valid_all;
    snapshot->safety.battery_voltage       = safety.battery_voltage;
    snapshot->safety.error_flags           = safety.error_flags;
    snapshot->safety.latched_error_flags   = safety.latched_error_flags;
    snapshot->safety.task_timeout_mask     = safety.task_timeout_mask;
    snapshot->safety.control_mode          = (uint8_t)control_mode.mode;
    snapshot->safety.current_control_valid = safety.current_control_valid;
    snapshot->safety.tim_break_active      = ((safety.error_flags & SYSTEM_ERROR_TIM_BREAK) != 0U) ? 1U : 0U;
    snapshot->safety.motor_fault_mask      = safety.motor_fault_mask;
    snapshot->current.invalid_reason_flags = power.invalid_reason_flags;
    snapshot->current.valid_flags          = power.valid_flags;
    snapshot->current.current_valid        = power.current_valid;

    snapshot->imu.online            = state.imu.online;
    snapshot->imu.chip_id           = state.imu.chip_id;
    snapshot->imu.calibrated        = state.imu.gyro_calibrated;
    snapshot->imu.sensor_time_valid = state.imu.sensor_time_valid;
    snapshot->imu.last_error        = state.imu.last_error;
    snapshot->imu.timestamp_ms      = state.imu.last_update_ms;
    snapshot->imu.sensor_time       = state.imu.sensor_time;
    snapshot->imu.sample_count      = state.imu.sample_count;
    snapshot->imu.quality_flags     = state.imu.quality_flags;
    snapshot->imu.roll_deg          = state.imu.roll_deg;
    snapshot->imu.pitch_deg         = state.imu.pitch_deg;
    snapshot->imu.yaw_deg           = state.imu.yaw_deg;
    snapshot->imu.temperature_c     = state.imu.temperature_c;
    snapshot->imu.calibration_state = AppSystemPublishSnapshot_MapImuCalibrationState(state.imu.gyro_auto_cal_state);
    snapshot->imu.calibration_fail_reason  = state.imu.gyro_cal_fail_reason;
    snapshot->imu.calibration_sample_count = state.imu.gyro_cal_sample_count;
    snapshot->imu.quality_counters[0]      = state.imu.spi_error_count;
    snapshot->imu.quality_counters[1]      = state.imu.init_failure_count;
    snapshot->imu.quality_counters[2]      = state.imu.fifo_overflow_count;
    snapshot->imu.quality_counters[3]      = state.imu.timestamp_error_count;
    snapshot->imu.quality_counters[4]      = state.imu.gyro_saturation_count;
    snapshot->imu.quality_counters[5]      = state.imu.accel_anomaly_count;
    snapshot->imu.quality_counters[6]      = state.imu.attitude_invalid_count;
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        snapshot->imu.accel_g[axis]  = state.imu.body_accel_g[axis];
        snapshot->imu.gyro_dps[axis] = state.imu.body_gyro_dps[axis];
    }
    for (uint8_t index = 0U; index < 4U; ++index)
    {
        snapshot->imu.quaternion[index] = state.imu.quaternion[index];
    }

    snapshot->control.emergency_stop                             = safety.emergency_stop;
    snapshot->control.fault_stop                                 = safety.fault_stop;
    snapshot->control.line_enabled                               = line.globally_enabled;
    snapshot->control.active_source                              = (uint8_t)command.active_source;
    snapshot->control.reset_reason_flags                         = system.reset_reason_flags;
    snapshot->communication.upper.checksum_errors                = upper.rx_checksum_errors;
    snapshot->communication.upper.timeout_resets                 = upper.rx_timeout_resets;
    snapshot->communication.upper.rx_overflows                   = upper.rx_overwrite_count;
    snapshot->communication.upper.tx_drops                       = upper.tx_busy_drops;
    snapshot->communication.upper.last_valid_frame_ms            = upper.last_valid_frame_ms;
    snapshot->communication.upper_last_rx_timestamp_ms           = HostCommunication_GetLastRxTimestamp();
    snapshot->communication.upper_session.session_id             = upper.session.session_id;
    snapshot->communication.upper_session.received_sequence      = upper.session.received_sequence;
    snapshot->communication.upper_session.applied_sequence       = upper.session.applied_sequence;
    snapshot->communication.upper_session.generation             = upper.session.generation;
    snapshot->communication.upper_session.last_valid_receive_ms  = upper.session.last_valid_receive_ms;
    snapshot->communication.upper_session.reject_reason          = upper.session.reject_reason;
    snapshot->communication.upper_session.ack_flags              = upper.session.ack_flags;
    snapshot->communication.esp12f.rx_frames                     = esp.rx_frames;
    snapshot->communication.esp12f.checksum_errors               = esp.rx_checksum_errors;
    snapshot->communication.esp12f.length_errors                 = esp.rx_length_errors;
    snapshot->communication.esp12f.timeout_resets                = esp.rx_timeout_resets;
    snapshot->communication.esp12f.rx_overflows                  = esp.rx_overflow_errors;
    snapshot->communication.esp12f.tx_drops                      = esp.tx_busy_drops;
    snapshot->communication.esp12f.last_valid_frame_ms           = esp.last_rx_timestamp_ms;
    snapshot->communication.esp12f.download_mode                 = esp.boot_mode_download;
    snapshot->communication.esp12f_session.session_id            = esp.session.session_id;
    snapshot->communication.esp12f_session.received_sequence     = esp.session.received_sequence;
    snapshot->communication.esp12f_session.applied_sequence      = esp.session.applied_sequence;
    snapshot->communication.esp12f_session.generation            = esp.session.generation;
    snapshot->communication.esp12f_session.last_valid_receive_ms = esp.session.last_valid_receive_ms;
    snapshot->communication.esp12f_session.reject_reason         = esp.session.reject_reason;
    snapshot->communication.esp12f_session.ack_flags             = esp.session.ack_flags;

    snapshot->modules.imu_online     = state.imu.online;
    snapshot->modules.encoder_online = state.wheel.speed_valid_all;
    snapshot->modules.motor_online   = ((safety.error_flags & SYSTEM_ERROR_DRV_FAULT) == 0U) ? 1U : 0U;
    snapshot->modules.adc_online     = power.current_valid;
    snapshot->modules.upper_online =
        AppSystemPublishSnapshot_IsFresh(now_ms,
                                         snapshot->communication.upper_last_rx_timestamp_ms,
                                         config->host_timeout_ms);
    snapshot->modules.esp12f_online =
        (esp.boot_mode_download == 0U)
            ? AppSystemPublishSnapshot_IsFresh(now_ms, esp.last_rx_timestamp_ms, config->esp12f_timeout_ms)
            : 0U;
    snapshot->modules.line_online =
        (line.sensor_valid != 0U)
            ? AppSystemPublishSnapshot_IsFresh(now_ms, line.sensor_timestamp_ms, config->line_timeout_ms)
            : 0U;
    snapshot->modules.ps2_online = teleoperation.online;
}
