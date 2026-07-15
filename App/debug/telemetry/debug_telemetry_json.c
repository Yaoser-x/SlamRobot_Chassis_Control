#include "debug_telemetry_json.h"

#include "debug_console_writer.h"
#include "debug_straight_telemetry.h"

#include <stdio.h>

#define DEBUG_TELEMETRY_JSON_TX_SIZE 1536U

void DebugTelemetryJson_Print(uint32_t now_ms, const debug_full_log_snapshot_t *snapshot)
{
    char                       tx[DEBUG_TELEMETRY_JSON_TX_SIZE];
    chassis_service_snapshot_t chassis = snapshot->chassis;
    imu_bmi270_state_t         imu     = snapshot->imu;
    adc_monitor_state_t        adc     = snapshot->adc;
    size_t                     pos     = (size_t)snprintf(tx,
                                  sizeof(tx),
                                  "{\"t_ms\":%lu,\"pid_error_mps\":[%.5f,%.5f,%.5f,%.5f],"
                                                          "\"pid_output_permille\":[%d,%d,%d,%d],\"actual_mps\":[%.5f,%.5f,%.5f,%.5f],"
                                                          "\"current_a\":[%.4f,%.4f,%.4f,%.4f],"
                                                          "\"imu_raw_acc\":[%d,%d,%d],\"imu_raw_gyro\":[%d,%d,%d],"
                                                          "\"imu_body_acc_g\":[%.5f,%.5f,%.5f],\"imu_filter_gyro_dps\":[%.5f,%.5f,%.5f],"
                                                          "\"euler_deg\":[%.4f,%.4f,%.4f],\"temperature_c\":%.3f,"
                                                          "\"imu_cal_state\":%u,\"imu_quality\":%lu",
                                  (unsigned long)now_ms,
                                  chassis.motor_error_mps[0],
                                  chassis.motor_error_mps[1],
                                  chassis.motor_error_mps[2],
                                  chassis.motor_error_mps[3],
                                  chassis.motor_output_permille[0],
                                  chassis.motor_output_permille[1],
                                  chassis.motor_output_permille[2],
                                  chassis.motor_output_permille[3],
                                  chassis.motor_actual_mps[0],
                                  chassis.motor_actual_mps[1],
                                  chassis.motor_actual_mps[2],
                                  chassis.motor_actual_mps[3],
                                  adc.current_a[0],
                                  adc.current_a[1],
                                  adc.current_a[2],
                                  adc.current_a[3],
                                  imu.accel_raw[0],
                                  imu.accel_raw[1],
                                  imu.accel_raw[2],
                                  imu.gyro_raw[0],
                                  imu.gyro_raw[1],
                                  imu.gyro_raw[2],
                                  imu.body_accel_g[0],
                                  imu.body_accel_g[1],
                                  imu.body_accel_g[2],
                                  imu.gyro_filtered_dps[0],
                                  imu.gyro_filtered_dps[1],
                                  imu.gyro_filtered_dps[2],
                                  imu.roll_deg,
                                  imu.pitch_deg,
                                  imu.yaw_deg,
                                  imu.temperature_c,
                                  imu.gyro_auto_cal_state,
                                  (unsigned long)imu.quality_flags);
    if (pos < sizeof(tx))
    {
        pos += DebugStraightTelemetry_FormatJson(tx + pos, sizeof(tx) - pos, &chassis, &adc);
        (void)snprintf(tx + pos, sizeof(tx) - pos, "}\r\n");
    }
    DebugConsoleWriter_Write(tx);
}
