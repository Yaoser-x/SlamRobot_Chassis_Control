#include "system_snapshot_service.h"

#include "chassis_layout.h"
#include "chassis_service.h"
#include "control_config.h"
#include "control_service.h"
#include "current_sensor_service.h"
#include "encoder_service.h"
#include "esp12f_service.h"
#include "imu_service.h"
#include "line_control_service.h"
#include "line_uart.h"
#include "motor_driver.h"
#include "platform_critical.h"
#include "ps2_control_service.h"
#include "reset_reason_service.h"
#include "safety_service.h"
#include "upper_uart_service.h"

static system_snapshot_t system_snapshots[2];
static uint8_t           active_snapshot;
static uint32_t          snapshot_generation;

static uint8_t SystemSnapshotService_IsFresh(uint32_t now_ms, uint32_t timestamp_ms, uint32_t timeout_ms)
{
    return (timestamp_ms != 0U && (uint32_t)(now_ms - timestamp_ms) <= timeout_ms) ? 1U : 0U;
}

void SystemSnapshotService_Init(void)
{
    platform_critical_state_t state = PlatformCritical_Enter();
    system_snapshots[0]             = (system_snapshot_t){0};
    system_snapshots[1]             = (system_snapshot_t){0};
    active_snapshot                 = 0U;
    snapshot_generation             = 0U;
    PlatformCritical_Exit(state);
}

void SystemSnapshotService_Update(uint32_t now_ms)
{
    system_snapshot_t           next = {0};
    chassis_service_snapshot_t  chassis;
    encoder_service_snapshot_t  encoder;
    current_sensor_snapshot_t   current;
    imu_service_snapshot_t      imu;
    motor_driver_state_t        motor;
    safety_service_snapshot_t   safety;
    upper_uart_service_state_t  upper;
    esp12f_service_state_t      esp;
    ps2_control_service_state_t ps2;
    line_sensor_data_t          line = {0};
    platform_critical_state_t   critical;
    uint8_t                     inactive;

    ChassisService_GetState(&chassis);
    EncoderService_GetSnapshot(&encoder);
    CurrentSensorService_GetSnapshot(&current);
    ImuService_GetSnapshot(&imu);
    MotorDriver_GetState(&motor);
    SafetyService_GetState(&safety);
    TaskHealthService_GetHealth(&next.task_health);
    POST_GetResult(&next.post);
    UpperUartService_GetState(&upper);
    Esp12fService_GetState(&esp);
    Ps2ControlService_GetState(&ps2);
    (void)LineUart_GetSensorData(&line);

    next.timestamp_ms = now_ms;
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        next.chassis.motor_target_mps[index]      = chassis.motor_target_mps[index];
        next.chassis.motor_actual_mps[index]      = chassis.motor_actual_mps[index];
        next.chassis.motor_output_permille[index] = motor.effective_pwm[index];
        next.encoder.count[index]                 = encoder.count[index];
        next.encoder.speed_mps[index]             = encoder.speed_mps[index];
        next.encoder.speed_valid[index]           = encoder.speed_valid[index];
        next.safety.motor_current_a[index]        = safety.motor_current_a[index];
        if (ChassisLayout_MotorEnabled((motor_id_t)index) != 0U)
        {
            next.chassis.motor_enabled_mask |= (uint8_t)(1U << index);
        }
        if (encoder.anomaly_count[index] > 0U)
        {
            next.encoder.anomaly_mask |= (uint8_t)(1U << index);
        }
        if (motor.fault_active[index] != 0U)
        {
            next.safety.motor_fault_mask |= (uint8_t)(1U << index);
        }
    }
    next.encoder.speed_valid_all      = encoder.speed_valid_all;
    next.safety.battery_voltage       = safety.battery_voltage;
    next.safety.error_flags           = safety.error_flags;
    next.safety.latched_error_flags   = safety.latched_error_flags;
    next.safety.task_timeout_mask     = safety.task_timeout_mask;
    next.safety.control_mode          = safety.control_mode;
    next.safety.current_control_valid = safety.current_control_valid;
    next.safety.tim_break_active      = ((safety.error_flags & SYSTEM_ERROR_TIM_BREAK) != 0U) ? 1U : 0U;
    next.current.invalid_reason_flags = current.invalid_reason_flags;
    next.current.valid_flags          = current.valid_flags;
    next.current.current_valid        = current.current_valid;

    next.imu.online                   = imu.online;
    next.imu.chip_id                  = imu.chip_id;
    next.imu.calibrated               = imu.gyro_calibrated;
    next.imu.sensor_time_valid        = imu.sensor_time_valid;
    next.imu.last_error               = imu.last_error;
    next.imu.timestamp_ms             = imu.last_update_ms;
    next.imu.sensor_time              = imu.sensor_time;
    next.imu.sample_count             = imu.sample_count;
    next.imu.quality_flags            = imu.quality_flags;
    next.imu.roll_deg                 = imu.roll_deg;
    next.imu.pitch_deg                = imu.pitch_deg;
    next.imu.yaw_deg                  = imu.yaw_deg;
    next.imu.temperature_c            = imu.temperature_c;
    next.imu.calibration_state        = imu.gyro_auto_cal_state;
    next.imu.calibration_fail_reason  = imu.gyro_cal_fail_reason;
    next.imu.calibration_sample_count = imu.gyro_cal_sample_count;
    next.imu.quality_counters[0]      = imu.spi_error_count;
    next.imu.quality_counters[1]      = imu.init_failure_count;
    next.imu.quality_counters[2]      = imu.fifo_overflow_count;
    next.imu.quality_counters[3]      = imu.timestamp_error_count;
    next.imu.quality_counters[4]      = imu.gyro_saturation_count;
    next.imu.quality_counters[5]      = imu.accel_anomaly_count;
    next.imu.quality_counters[6]      = imu.attitude_invalid_count;
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        next.imu.accel_g[axis]  = imu.body_accel_g[axis];
        next.imu.gyro_dps[axis] = imu.body_gyro_dps[axis];
    }
    for (uint8_t index = 0U; index < 4U; ++index)
    {
        next.imu.quaternion[index] = imu.quaternion[index];
    }

    next.control.emergency_stop     = ControlService_IsEmergencyStop();
    next.control.fault_stop         = ControlService_IsFaultStop();
    next.control.line_enabled       = LineControlService_IsEnabled();
    next.control.active_source      = ControlService_GetActiveSource();
    next.control.reset_reason_flags = ResetReasonService_GetFlags();

    next.communication.upper.checksum_errors      = upper.rx_checksum_errors;
    next.communication.upper.timeout_resets       = upper.rx_timeout_resets;
    next.communication.upper.rx_overflows         = upper.rx_overwrite_count;
    next.communication.upper.tx_drops             = upper.tx_busy_drops;
    next.communication.upper.last_valid_frame_ms  = upper.last_valid_frame_ms;
    next.communication.upper_last_rx_timestamp_ms = UpperUartService_GetLastRxTimestamp();
    next.communication.esp12f.rx_frames           = esp.rx_frames;
    next.communication.esp12f.checksum_errors     = esp.rx_checksum_errors;
    next.communication.esp12f.length_errors       = esp.rx_length_errors;
    next.communication.esp12f.timeout_resets      = esp.rx_timeout_resets;
    next.communication.esp12f.rx_overflows        = esp.rx_overflow_errors;
    next.communication.esp12f.tx_drops            = esp.tx_busy_drops;
    next.communication.esp12f.last_valid_frame_ms = esp.last_rx_timestamp_ms;
    next.communication.esp12f.download_mode       = esp.boot_mode_download;

    next.modules.imu_online     = imu.online;
    next.modules.encoder_online = encoder.speed_valid_all;
    next.modules.motor_online   = ((safety.error_flags & SYSTEM_ERROR_DRV_FAULT) == 0U) ? 1U : 0U;
    next.modules.adc_online     = current.current_valid;
    next.modules.upper_online   = SystemSnapshotService_IsFresh(now_ms,
                                                              next.communication.upper_last_rx_timestamp_ms,
                                                              OLED_MODULE_TIMEOUT_RPI_MS);
    next.modules.esp12f_online =
        (esp.boot_mode_download == 0U)
            ? SystemSnapshotService_IsFresh(now_ms, esp.last_rx_timestamp_ms, CONTROL_TIMEOUT_ESP12F_MS)
            : 0U;
    next.modules.line_online =
        (line.valid != 0U) ? SystemSnapshotService_IsFresh(now_ms, line.timestamp_ms, OLED_MODULE_TIMEOUT_LINE_MS) : 0U;
    next.modules.ps2_online = ps2.online;

    inactive                   = (uint8_t)(active_snapshot ^ 1U);
    system_snapshots[inactive] = next;
    critical                   = PlatformCritical_Enter();
    snapshot_generation++;
    system_snapshots[inactive].generation = snapshot_generation;
    active_snapshot                       = inactive;
    PlatformCritical_Exit(critical);
}

uint32_t SystemSnapshotService_Get(system_snapshot_t *out)
{
    platform_critical_state_t state;
    uint32_t                  generation;

    if (out == 0)
    {
        return 0U;
    }
    state      = PlatformCritical_Enter();
    *out       = system_snapshots[active_snapshot];
    generation = out->generation;
    PlatformCritical_Exit(state);
    return generation;
}
