#include "telemetry_frame_builder.h"

#include "upper_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint16_t
ExpectedStatus(const communication_publish_model_t *snapshot, uint8_t comm_health_flags, uint8_t *out, uint16_t out_len)
{
    upper_status_payload_t status = {0};
    uint8_t                payload[UPPER_PROTOCOL_STATUS_PAYLOAD_LEN];
    uint8_t                payload_len;

    status.battery_voltage     = snapshot->safety.battery_voltage;
    status.error_flags         = snapshot->safety.error_flags;
    status.latched_error_flags = snapshot->safety.latched_error_flags;
    status.status_flags        = UPPER_STATUS_FLAG_ESTOP | UPPER_STATUS_FLAG_FAULT_STOP | UPPER_STATUS_FLAG_LINE_ENABLED
                          | UPPER_STATUS_FLAG_SPEED_VALID_ALL;
    status.control_source         = snapshot->safety.control_mode;
    status.motor_enabled_mask     = snapshot->chassis.motor_enabled_mask;
    status.motor_speed_valid_mask = 0x05U;
    status.encoder_anomaly_mask   = snapshot->encoder.anomaly_mask;
    status.comm_health_flags      = comm_health_flags;
    for (uint8_t index = 0U; index < UPPER_PROTOCOL_MOTOR_COUNT; ++index)
    {
        status.motor_speed_mps[index]       = snapshot->chassis.motor_actual_mps[index];
        status.encoder_count[index]         = snapshot->encoder.count[index];
        status.motor_current_a[index]       = snapshot->safety.motor_current_a[index];
        status.motor_target_mps[index]      = snapshot->chassis.motor_target_mps[index];
        status.motor_output_permille[index] = snapshot->chassis.motor_output_permille[index];
    }
    payload_len = UpperProtocol_BuildStatusPayload(&status, payload, sizeof(payload));
    return UpperProtocol_BuildFrame(UPPER_CMD_STATUS, payload, payload_len, out, out_len);
}

static void SeedSnapshot(communication_publish_model_t *snapshot)
{
    *snapshot                              = (communication_publish_model_t){0};
    snapshot->safety.battery_voltage       = 12.6f;
    snapshot->safety.error_flags           = 0x12345678UL;
    snapshot->safety.latched_error_flags   = 0x87654321UL;
    snapshot->safety.control_mode          = 3U;
    snapshot->safety.task_timeout_mask     = 0x55AAU;
    snapshot->control.emergency_stop       = 1U;
    snapshot->control.fault_stop           = 1U;
    snapshot->control.line_enabled         = 1U;
    snapshot->control.reset_reason_flags   = 0xCAFEBABEU;
    snapshot->chassis.motor_enabled_mask   = 0x0FU;
    snapshot->encoder.speed_valid_all      = 1U;
    snapshot->encoder.speed_valid[0]       = 1U;
    snapshot->encoder.speed_valid[2]       = 1U;
    snapshot->encoder.anomaly_mask         = 0x02U;
    snapshot->post.done                    = 1U;
    snapshot->post.error_flags             = 0x10203040UL;
    snapshot->current.invalid_reason_flags = 0x11223344UL;
    snapshot->imu.online                   = 1U;
    snapshot->imu.calibrated               = 1U;
    snapshot->imu.sensor_time_valid        = 1U;
    snapshot->imu.quality_flags            = 0x01020304UL;
    snapshot->imu.sensor_time              = 123456U;
    snapshot->imu.sample_count             = 77U;
    snapshot->imu.temperature_c            = -3.6f;
    snapshot->imu.roll_deg                 = 1.0f;
    snapshot->imu.pitch_deg                = 2.0f;
    snapshot->imu.yaw_deg                  = 3.0f;
    for (uint8_t index = 0U; index < UPPER_PROTOCOL_MOTOR_COUNT; ++index)
    {
        snapshot->chassis.motor_actual_mps[index]      = (float)index + 0.1f;
        snapshot->chassis.motor_target_mps[index]      = (float)index + 0.2f;
        snapshot->chassis.motor_output_permille[index] = (int16_t)(100 + index);
        snapshot->encoder.count[index]                 = 1000 + index;
        snapshot->safety.motor_current_a[index]        = (float)index + 0.3f;
        snapshot->imu.quaternion[index]                = (float)index + 0.4f;
    }
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        snapshot->imu.accel_g[axis]  = (float)axis + 0.5f;
        snapshot->imu.gyro_dps[axis] = (float)axis + 0.6f;
    }
    for (uint8_t index = 0U; index < 7U; ++index)
    {
        snapshot->imu.quality_counters[index] = 10U + index;
    }
}

int main(void)
{
    communication_publish_model_t snapshot;
    upper_diagnostic_payload_t    diagnostic = {0};
    upper_imu_status_payload_t    imu        = {0};
    uint8_t                       payload[UPPER_PROTOCOL_IMU_STATUS_PAYLOAD_LEN];
    uint8_t                       payload_len;
    uint8_t                       actual[UPPER_PROTOCOL_MAX_FRAME];
    uint8_t                       expected[UPPER_PROTOCOL_MAX_FRAME];
    uint16_t                      actual_len;
    uint16_t                      expected_len;

    SeedSnapshot(&snapshot);
    snapshot.communication.upper.checksum_errors = 1U;
    snapshot.communication.upper.timeout_resets  = 1U;
    snapshot.communication.upper.tx_drops        = 1U;
    actual_len   = TelemetryFrameBuilder_BuildStatus(&snapshot, COMMUNICATION_LINK_UPPER, actual, sizeof(actual));
    expected_len = ExpectedStatus(&snapshot,
                                  UPPER_COMM_HEALTH_CRC_ERR | UPPER_COMM_HEALTH_TIMEOUT | UPPER_COMM_HEALTH_TX_DROP,
                                  expected,
                                  sizeof(expected));
    assert(actual_len == expected_len && memcmp(actual, expected, actual_len) == 0);

    snapshot.communication.esp12f.checksum_errors = 1U;
    snapshot.communication.esp12f.rx_overflows    = 1U;
    snapshot.communication.esp12f.tx_drops        = 1U;
    actual_len = TelemetryFrameBuilder_BuildStatus(&snapshot, COMMUNICATION_LINK_ESP12F, actual, sizeof(actual));
    expected_len =
        ExpectedStatus(&snapshot,
                       UPPER_COMM_HEALTH_ESP_CRC | UPPER_COMM_HEALTH_ESP_TIMEOUT | UPPER_COMM_HEALTH_ESP_TX_DROP,
                       expected,
                       sizeof(expected));
    assert(actual_len == expected_len && memcmp(actual, expected, actual_len) == 0);

    actual_len           = TelemetryFrameBuilder_BuildDiagnostic(&snapshot, 9000U, actual, sizeof(actual));
    diagnostic.post_done = snapshot.post.done;
    diagnostic.imu_status_flags =
        UPPER_IMU_FLAG_ONLINE | UPPER_IMU_FLAG_CALIBRATED | UPPER_IMU_FLAG_ERROR | UPPER_IMU_FLAG_SENSOR_TIME;
    diagnostic.post_error_flags         = snapshot.post.error_flags;
    diagnostic.adc_invalid_reason_flags = snapshot.current.invalid_reason_flags;
    diagnostic.task_timeout_mask        = snapshot.safety.task_timeout_mask;
    diagnostic.imu_quality_flags        = snapshot.imu.quality_flags;
    diagnostic.reset_reason_flags       = snapshot.control.reset_reason_flags;
    diagnostic.uptime_ms                = 9000U;
    payload_len                         = UpperProtocol_BuildDiagnosticPayload(&diagnostic, payload, sizeof(payload));
    expected_len = UpperProtocol_BuildFrame(UPPER_CMD_DIAGNOSTIC, payload, payload_len, expected, sizeof(expected));
    assert(actual_len == expected_len && memcmp(actual, expected, actual_len) == 0);

    actual_len = TelemetryFrameBuilder_BuildImu(&snapshot, 9000U, actual, sizeof(actual));
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        imu.accel_g[axis]            = snapshot.imu.accel_g[axis];
        imu.gyro_corrected_dps[axis] = snapshot.imu.gyro_dps[axis];
    }
    imu.euler_deg[0] = snapshot.imu.roll_deg;
    imu.euler_deg[1] = snapshot.imu.pitch_deg;
    imu.euler_deg[2] = snapshot.imu.yaw_deg;
    for (uint8_t index = 0U; index < 4U; ++index)
    {
        imu.quaternion[index] = snapshot.imu.quaternion[index];
    }
    imu.timestamp_ms  = 9000U;
    imu.sensor_time   = snapshot.imu.sensor_time;
    imu.sample_count  = snapshot.imu.sample_count;
    imu.quality_flags = snapshot.imu.quality_flags;
    for (uint8_t index = 0U; index < 7U; ++index)
    {
        imu.quality_counters[index] = snapshot.imu.quality_counters[index];
    }
    imu.status_flags  = diagnostic.imu_status_flags;
    imu.temperature_c = -4;
    payload_len       = UpperProtocol_BuildImuStatusPayload(&imu, payload, sizeof(payload));
    expected_len = UpperProtocol_BuildFrame(UPPER_CMD_IMU_STATUS, payload, payload_len, expected, sizeof(expected));
    assert(actual_len == expected_len && memcmp(actual, expected, actual_len) == 0);
    snapshot.imu.online = 0U;
    assert(TelemetryFrameBuilder_BuildImu(&snapshot, 9000U, actual, sizeof(actual)) == 0U);

    puts("PASS: telemetry frame builder");
    return 0;
}
