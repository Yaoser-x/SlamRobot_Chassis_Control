#include "robot_link_protocol.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

static void print_hex(const uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0U; i < length; ++i)
    {
        (void)printf("%02x", data[i]);
    }
}

static void
print_frame(const char *name, uint8_t command, const uint8_t *payload, uint8_t payload_length, uint8_t is_last)
{
    uint8_t  frame[ROBOT_LINK_PROTOCOL_MAX_FRAME] = {0U};
    uint16_t frame_length = UpperProtocol_BuildFrame(command, payload, payload_length, frame, sizeof(frame));

    (void)printf("    {\"name\":\"%s\",\"command\":%u,\"payload_length\":%u,"
                 "\"frame_length\":%u,\"frame_hex\":\"",
                 name,
                 command,
                 payload_length,
                 frame_length);
    print_hex(frame, frame_length);
    (void)printf("\"}%s\n", is_last != 0U ? "" : ",");
}

static void emit_status(void)
{
    upper_status_payload_t status                                          = {0};
    uint8_t                payload[ROBOT_LINK_PROTOCOL_STATUS_PAYLOAD_LEN] = {0U};

    status.battery_voltage          = 12.345f;
    status.motor_speed_mps[0]       = 1.25f;
    status.motor_speed_mps[1]       = -0.5f;
    status.motor_speed_mps[2]       = 0.001f;
    status.motor_speed_mps[3]       = -2.0f;
    status.encoder_count[0]         = 1;
    status.encoder_count[1]         = -2;
    status.encoder_count[2]         = INT32_MAX;
    status.encoder_count[3]         = INT32_MIN;
    status.motor_current_a[0]       = 0.1f;
    status.motor_current_a[1]       = 1.2f;
    status.motor_current_a[2]       = 2.4f;
    status.motor_current_a[3]       = 65.535f;
    status.motor_target_mps[0]      = 0.25f;
    status.motor_target_mps[1]      = -0.25f;
    status.motor_target_mps[2]      = 1.5f;
    status.motor_target_mps[3]      = -1.5f;
    status.motor_output_permille[0] = 100;
    status.motor_output_permille[1] = -100;
    status.motor_output_permille[2] = 1000;
    status.motor_output_permille[3] = -1000;
    status.error_flags              = 0x12345678UL;
    status.latched_error_flags      = 0x90ABCDEFUL;
    status.status_flags             = 0x0FU;
    status.control_source           = 3U;
    status.motor_enabled_mask       = 0x0FU;
    status.motor_speed_valid_mask   = 0x05U;
    status.encoder_anomaly_mask     = 0x0AU;
    status.comm_health_flags        = 0x15U;
    status.status_sequence          = 0x01020304UL;
    status.timestamp_ms             = 0x11121314UL;
    status.session_id               = 0x2122232425262728ULL;
    status.received_sequence        = 0x31323334UL;
    status.applied_sequence         = 0x41424344UL;
    status.reject_reason            = 6U;
    status.side_consistency_flags   = 2U;
    status.ack_flags                = 0x13U;

    (void)UpperProtocol_BuildStatusPayload(&status, payload, sizeof(payload));
    print_frame("status_v3", UPPER_CMD_STATUS, payload, sizeof(payload), 0U);
}

static void emit_hello(void)
{
    upper_hello_payload_t hello = {
        .schema_version = 1U,
        .identity =
            {
                .hardware_revision = 0x00020000UL,
                .capabilities      = COMMUNICATION_REQUIRED_CAPABILITIES,
            },
        .parameter_crc32 = 0xA1B2C3D4UL,
    };
    uint8_t payload[ROBOT_LINK_PROTOCOL_HELLO_PAYLOAD_LEN] = {0U};

    for (uint8_t i = 0U; i < COMMUNICATION_GIT_COMMIT_LENGTH; ++i)
    {
        hello.identity.git_commit[i] = i;
    }
    (void)UpperProtocol_BuildHelloPayload(&hello, payload, sizeof(payload));
    print_frame("hello_v3", UPPER_CMD_HELLO, payload, sizeof(payload), 0U);
}

static void emit_layout_status(const char *name, uint8_t enabled_mask, uint8_t valid_mask, uint8_t anomaly_mask)
{
    upper_status_payload_t status                                          = {0};
    uint8_t                payload[ROBOT_LINK_PROTOCOL_STATUS_PAYLOAD_LEN] = {0U};

    status.motor_enabled_mask     = enabled_mask;
    status.motor_speed_valid_mask = valid_mask;
    status.encoder_anomaly_mask   = anomaly_mask;
    status.status_sequence        = 7U;
    status.timestamp_ms           = 50U;
    if (enabled_mask != 0U && enabled_mask == valid_mask)
    {
        status.status_flags = UPPER_STATUS_FLAG_SPEED_VALID_ALL;
    }
    (void)UpperProtocol_BuildStatusPayload(&status, payload, sizeof(payload));
    print_frame(name, UPPER_CMD_STATUS, payload, sizeof(payload), 0U);
}

static void emit_imu(const char *name, int8_t temperature_c, uint8_t is_last)
{
    upper_imu_status_payload_t imu                                                 = {0};
    uint8_t                    payload[ROBOT_LINK_PROTOCOL_IMU_STATUS_PAYLOAD_LEN] = {0U};

    imu.accel_g[0]            = 0.125f;
    imu.accel_g[1]            = -0.25f;
    imu.accel_g[2]            = 1.0f;
    imu.gyro_corrected_dps[0] = 1.5f;
    imu.gyro_corrected_dps[1] = -2.5f;
    imu.gyro_corrected_dps[2] = 3.5f;
    imu.euler_deg[0]          = 10.0f;
    imu.euler_deg[1]          = -20.0f;
    imu.euler_deg[2]          = 30.0f;
    imu.quaternion[0]         = 1.0f;
    imu.quaternion[1]         = 0.0f;
    imu.quaternion[2]         = 0.0f;
    imu.quaternion[3]         = 0.0f;
    imu.timestamp_ms          = 0x01020304UL;
    imu.sensor_time           = 0x05060708UL;
    imu.sample_count          = 0x11121314UL;
    imu.quality_flags         = 0x21222324UL;
    for (uint8_t i = 0U; i < 7U; ++i)
    {
        imu.quality_counters[i] = (uint32_t)i + 1UL;
    }
    imu.status_flags  = 0x0BU;
    imu.temperature_c = temperature_c;

    (void)UpperProtocol_BuildImuStatusPayload(&imu, payload, sizeof(payload));
    print_frame(name, UPPER_CMD_IMU_STATUS, payload, sizeof(payload), is_last);
}

static void emit_diagnostic(void)
{
    upper_diagnostic_payload_t diagnostic                                          = {0};
    uint8_t                    payload[ROBOT_LINK_PROTOCOL_DIAGNOSTIC_PAYLOAD_LEN] = {0U};

    diagnostic.post_done                = 1U;
    diagnostic.imu_status_flags         = 0x0BU;
    diagnostic.post_error_flags         = 0x01020304UL;
    diagnostic.adc_invalid_reason_flags = 0x11121314UL;
    diagnostic.task_timeout_mask        = 0x0123U;
    diagnostic.imu_quality_flags        = 0x21222324UL;
    diagnostic.reset_reason_flags       = 0x31323334UL;
    diagnostic.uptime_ms                = 0x41424344UL;
    (void)UpperProtocol_BuildDiagnosticPayload(&diagnostic, payload, sizeof(payload));
    print_frame("diagnostic_v1", UPPER_CMD_DIAGNOSTIC, payload, sizeof(payload), 0U);
}

int main(void)
{
    (void)printf("{\n  \"schema\":1,\n  \"protocol_version\":3,\n  \"frames\":[\n");
    emit_hello();
    emit_status();
    emit_layout_status("status_default_2wd", 0x06U, 0x06U, 0x00U);
    emit_layout_status("status_4wd", 0x0FU, 0x0FU, 0x00U);
    emit_layout_status("status_single_wheel_anomaly", 0x06U, 0x02U, 0x04U);
    emit_diagnostic();
    emit_imu("imu_temp_23c", 23, 0U);
    emit_imu("imu_temp_minus_41c", -41, 0U);
    emit_imu("imu_temp_87c", 87, 1U);
    (void)printf("  ]\n}\n");
    return 0;
}
