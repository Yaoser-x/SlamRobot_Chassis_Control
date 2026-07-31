#include "debug_telemetry_csv.h"

#include "debug_console_writer.h"
#include "debug_straight_telemetry.h"

#include <stdio.h>

#define DEBUG_TELEMETRY_CSV_TX_SIZE 1536U

static int32_t DebugTelemetryCsv_Milli(float value)
{
    return (int32_t)(value * 1000.0f);
}

void DebugTelemetryCsv_Print(uint32_t now_ms, const debug_full_log_snapshot_t *snapshot)
{
    char                           tx[DEBUG_TELEMETRY_CSV_TX_SIZE];
    power_adc_driver_state_t       adc_state     = snapshot->adc;
    motion_control_status_t        chassis_state = snapshot->chassis;
    safety_management_status_t     monitor_state = snapshot->monitor;
    motor_driver_state_t           motor_state   = snapshot->motor;
    state_estimation_imu_status_t  imu_state     = snapshot->imu;
    teleoperation_status_t         ps2_state     = snapshot->ps2;
    line_sensor_driver_state_t     line_state    = snapshot->line;
    wireless_communication_state_t esp_state     = snapshot->esp;
    float                          motor_log_speed_mps[MOTOR_ID_COUNT];

    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        motor_log_speed_mps[i] = snapshot->motor_log_speed_mps[i];
    }

    size_t pos = (size_t)snprintf(tx,
                                  sizeof(tx),
                                  "%lu,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%ld,%ld,%ld,%ld,%ld,%u,%u,%lu,%u,%lu,%lu,%lu,%lu,%"
                                  "lu,%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%lu,%.4f,%.4f,%."
                                  "4f,%.4f,%lu,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%d,%.2f,%u\r\n",
                                  (unsigned long)now_ms,
                                  (long)DebugTelemetryCsv_Milli(motor_log_speed_mps[MOTOR_ID_M1]),
                                  (long)DebugTelemetryCsv_Milli(motor_log_speed_mps[MOTOR_ID_M2]),
                                  (long)DebugTelemetryCsv_Milli(motor_log_speed_mps[MOTOR_ID_M3]),
                                  (long)DebugTelemetryCsv_Milli(motor_log_speed_mps[MOTOR_ID_M4]),
                                  motor_state.effective_pwm[MOTOR_ID_M1],
                                  motor_state.effective_pwm[MOTOR_ID_M2],
                                  motor_state.effective_pwm[MOTOR_ID_M3],
                                  motor_state.effective_pwm[MOTOR_ID_M4],
                                  (long)DebugTelemetryCsv_Milli(adc_state.battery_voltage),
                                  (long)DebugTelemetryCsv_Milli(adc_state.current_a[MOTOR_ID_M1]),
                                  (long)DebugTelemetryCsv_Milli(adc_state.current_a[MOTOR_ID_M2]),
                                  (long)DebugTelemetryCsv_Milli(adc_state.current_a[MOTOR_ID_M3]),
                                  (long)DebugTelemetryCsv_Milli(adc_state.current_a[MOTOR_ID_M4]),
                                  imu_state.online,
                                  imu_state.chip_id,
                                  (unsigned long)monitor_state.error_flags,
                                  monitor_state.active_source,
                                  (unsigned long)ps2_state.rx_ok_count,
                                  (unsigned long)ps2_state.rx_fail_count,
                                  (unsigned long)line_state.rx_bytes,
                                  (unsigned long)line_state.rx_frames,
                                  (unsigned long)esp_state.rx_frames,
                                  (unsigned long)esp_state.tx_frames,
                                  imu_state.body_accel_g[0],
                                  imu_state.body_accel_g[1],
                                  imu_state.body_accel_g[2],
                                  imu_state.body_gyro_dps[0],
                                  imu_state.body_gyro_dps[1],
                                  imu_state.body_gyro_dps[2],
                                  imu_state.gyro_filtered_dps[0],
                                  imu_state.gyro_filtered_dps[1],
                                  imu_state.gyro_filtered_dps[2],
                                  imu_state.roll_deg,
                                  imu_state.pitch_deg,
                                  imu_state.yaw_deg,
                                  (unsigned long)imu_state.sensor_time,
                                  imu_state.quaternion[0],
                                  imu_state.quaternion[1],
                                  imu_state.quaternion[2],
                                  imu_state.quaternion[3],
                                  (unsigned long)imu_state.quality_flags,
                                  chassis_state.motor_error_mps[0],
                                  chassis_state.motor_error_mps[1],
                                  chassis_state.motor_error_mps[2],
                                  chassis_state.motor_error_mps[3],
                                  chassis_state.motor_output_permille[0],
                                  chassis_state.motor_output_permille[1],
                                  chassis_state.motor_output_permille[2],
                                  chassis_state.motor_output_permille[3],
                                  imu_state.temperature_c,
                                  imu_state.gyro_auto_cal_state);
    if (pos < sizeof(tx))
    {
        tx[pos - 2U] = ',';
        pos--;
        pos += DebugStraightTelemetry_FormatCsv(tx + pos, sizeof(tx) - pos, &chassis_state, &adc_state);
        (void)snprintf(tx + pos, sizeof(tx) - pos, "\r\n");
    }
    DebugConsoleWriter_Write(tx);
}
