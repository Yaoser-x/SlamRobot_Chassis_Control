#include "telemetry_encoder.h"

#include "robot_link_protocol.h"

#define WIRE_IMU_QUALITY_GYRO_INVALID         (1UL << 0)
#define WIRE_IMU_QUALITY_ACCEL_INVALID        (1UL << 1)
#define WIRE_IMU_QUALITY_ORIENTATION_INVALID  (1UL << 2)
#define WIRE_IMU_QUALITY_TIMESTAMP_INVALID    (1UL << 3)
#define WIRE_IMU_QUALITY_GYRO_WARNING         (1UL << 4)
#define WIRE_IMU_QUALITY_ACCEL_WARNING        (1UL << 5)
#define WIRE_IMU_QUALITY_ORIENTATION_WARNING  (1UL << 6)
#define WIRE_IMU_QUALITY_TIMESTAMP_WARNING    (1UL << 7)
#define WIRE_IMU_QUALITY_FATAL                (1UL << 8)

#define INTERNAL_IMU_QUALITY_SPI_ERROR        (1UL << 0)
#define INTERNAL_IMU_QUALITY_INIT_FAILED      (1UL << 1)
#define INTERNAL_IMU_QUALITY_FIFO_OVERFLOW    (1UL << 2)
#define INTERNAL_IMU_QUALITY_TIMESTAMP_ERROR  (1UL << 3)
#define INTERNAL_IMU_QUALITY_GYRO_SATURATION  (1UL << 4)
#define INTERNAL_IMU_QUALITY_ACCEL_ANOMALY    (1UL << 5)
#define INTERNAL_IMU_QUALITY_ATTITUDE_INVALID (1UL << 6)
#define INTERNAL_IMU_QUALITY_POLL_FALLBACK    (1UL << 7)
#define INTERNAL_IMU_QUALITY_PROFILE_MISMATCH (1UL << 8)

static uint32_t TelemetryEncoder_MapImuQuality(const communication_publish_imu_t *imu)
{
    uint32_t wire = 0UL;
    uint32_t fatal =
        INTERNAL_IMU_QUALITY_SPI_ERROR | INTERNAL_IMU_QUALITY_INIT_FAILED | INTERNAL_IMU_QUALITY_FIFO_OVERFLOW;

    if ((imu->quality_flags & fatal) != 0UL || imu->online == 0U || imu->last_error != 0U)
    {
        wire |= WIRE_IMU_QUALITY_GYRO_INVALID | WIRE_IMU_QUALITY_ACCEL_INVALID | WIRE_IMU_QUALITY_ORIENTATION_INVALID
                | WIRE_IMU_QUALITY_FATAL;
    }
    if ((imu->quality_flags & INTERNAL_IMU_QUALITY_TIMESTAMP_ERROR) != 0UL || imu->sensor_time_valid == 0U)
    {
        wire |= WIRE_IMU_QUALITY_TIMESTAMP_INVALID;
    }
    if ((imu->quality_flags & INTERNAL_IMU_QUALITY_GYRO_SATURATION) != 0UL)
    {
        wire |= WIRE_IMU_QUALITY_GYRO_WARNING;
    }
    if ((imu->quality_flags & INTERNAL_IMU_QUALITY_ACCEL_ANOMALY) != 0UL)
    {
        wire |= WIRE_IMU_QUALITY_ACCEL_WARNING;
    }
    if ((imu->quality_flags & INTERNAL_IMU_QUALITY_ATTITUDE_INVALID) != 0UL)
    {
        wire |= WIRE_IMU_QUALITY_ORIENTATION_INVALID;
    }
    if ((imu->quality_flags & INTERNAL_IMU_QUALITY_PROFILE_MISMATCH) != 0UL || imu->calibrated == 0U)
    {
        wire |= WIRE_IMU_QUALITY_ORIENTATION_WARNING;
    }
    if ((imu->quality_flags & INTERNAL_IMU_QUALITY_POLL_FALLBACK) != 0UL)
    {
        wire |= WIRE_IMU_QUALITY_TIMESTAMP_WARNING;
    }
    return wire;
}

static uint8_t TelemetryEncoder_ImuFlags(const communication_publish_imu_t *imu)
{
    uint8_t  flags        = 0U;
    uint32_t wire_quality = TelemetryEncoder_MapImuQuality(imu);

    if (imu->online != 0U)
    {
        flags |= UPPER_IMU_FLAG_ONLINE;
    }
    if (imu->calibrated != 0U)
    {
        flags |= UPPER_IMU_FLAG_CALIBRATED;
    }
    if ((wire_quality
         & (WIRE_IMU_QUALITY_GYRO_INVALID | WIRE_IMU_QUALITY_ACCEL_INVALID | WIRE_IMU_QUALITY_ORIENTATION_INVALID
            | WIRE_IMU_QUALITY_TIMESTAMP_INVALID | WIRE_IMU_QUALITY_FATAL))
        != 0UL)
    {
        flags |= UPPER_IMU_FLAG_ERROR;
    }
    if (imu->sensor_time_valid != 0U)
    {
        flags |= UPPER_IMU_FLAG_SENSOR_TIME;
    }
    return flags;
}

static uint8_t TelemetryEncoder_SideConsistency(const communication_publish_model_t *snapshot)
{
    static const uint8_t side_first[2]  = {0U, 2U};
    static const uint8_t side_second[2] = {1U, 3U};
    uint8_t              flags          = 0U;

    for (uint8_t side = 0U; side < 2U; ++side)
    {
        uint8_t first     = side_first[side];
        uint8_t second    = side_second[side];
        uint8_t pair_mask = (uint8_t)((1U << first) | (1U << second));

        if ((snapshot->chassis.motor_enabled_mask & pair_mask) == pair_mask
            && snapshot->encoder.speed_valid[first] != 0U && snapshot->encoder.speed_valid[second] != 0U)
        {
            float delta = snapshot->encoder.speed_mps[first] - snapshot->encoder.speed_mps[second];
            if (delta < 0.0f)
            {
                delta = -delta;
            }
            if (delta > 0.10f)
            {
                flags |= (uint8_t)(1U << side);
            }
        }
    }
    return flags;
}

uint16_t TelemetryEncoder_BuildHello(const communication_firmware_identity_t *identity,
                                     uint32_t                                 parameter_crc32,
                                     uint8_t                                 *out,
                                     uint16_t                                 out_len)
{
    upper_hello_payload_t hello = {
        .schema_version  = 1U,
        .parameter_crc32 = parameter_crc32,
    };
    uint8_t payload[ROBOT_LINK_PROTOCOL_HELLO_PAYLOAD_LEN];
    uint8_t payload_len;

    if (identity == 0 || out == 0)
    {
        return 0U;
    }
    hello.identity = *identity;
    payload_len    = UpperProtocol_BuildHelloPayload(&hello, payload, sizeof(payload));
    return UpperProtocol_BuildFrame(UPPER_CMD_HELLO, payload, payload_len, out, out_len);
}

uint16_t TelemetryEncoder_BuildStatus(const communication_publish_model_t *snapshot,
                                      communication_link_t                 link,
                                      uint8_t                             *out,
                                      uint16_t                             out_len)
{
    upper_status_payload_t status = {0};
    uint8_t                payload[ROBOT_LINK_PROTOCOL_STATUS_PAYLOAD_LEN];
    uint8_t                payload_len;

    if (snapshot == 0 || out == 0)
    {
        return 0U;
    }
    status.battery_voltage     = snapshot->safety.battery_voltage;
    status.error_flags         = snapshot->safety.error_flags;
    status.latched_error_flags = snapshot->safety.latched_error_flags;
    status.control_source      = snapshot->control.active_source;
    if (snapshot->control.emergency_stop != 0U)
    {
        status.status_flags |= UPPER_STATUS_FLAG_ESTOP;
    }
    if (snapshot->control.fault_stop != 0U)
    {
        status.status_flags |= UPPER_STATUS_FLAG_FAULT_STOP;
    }
    if (snapshot->control.line_enabled != 0U)
    {
        status.status_flags |= UPPER_STATUS_FLAG_LINE_ENABLED;
    }
    if (snapshot->encoder.speed_valid_all != 0U)
    {
        status.status_flags |= UPPER_STATUS_FLAG_SPEED_VALID_ALL;
    }
    for (uint8_t index = 0U; index < ROBOT_LINK_PROTOCOL_MOTOR_COUNT; ++index)
    {
        status.motor_speed_mps[index]       = snapshot->chassis.motor_actual_mps[index];
        status.encoder_count[index]         = snapshot->encoder.count[index];
        status.motor_current_a[index]       = snapshot->safety.motor_current_a[index];
        status.motor_target_mps[index]      = snapshot->chassis.motor_target_mps[index];
        status.motor_output_permille[index] = snapshot->chassis.motor_output_permille[index];
        if ((snapshot->chassis.motor_enabled_mask & (uint8_t)(1U << index)) != 0U)
        {
            status.motor_enabled_mask |= (uint8_t)(1U << index);
        }
        if (snapshot->encoder.speed_valid[index] != 0U)
        {
            status.motor_speed_valid_mask |= (uint8_t)(1U << index);
        }
    }
    status.encoder_anomaly_mask   = snapshot->encoder.anomaly_mask;
    status.status_sequence        = snapshot->generation;
    status.timestamp_ms           = snapshot->timestamp_ms;
    status.side_consistency_flags = TelemetryEncoder_SideConsistency(snapshot);

    if (link == COMMUNICATION_LINK_ESP12F)
    {
        status.session_id        = snapshot->communication.esp12f_session.session_id;
        status.received_sequence = snapshot->communication.esp12f_session.received_sequence;
        status.applied_sequence  = snapshot->communication.esp12f_session.applied_sequence;
        status.reject_reason     = snapshot->communication.esp12f_session.reject_reason;
        status.ack_flags         = snapshot->communication.esp12f_session.ack_flags;
    }
    else
    {
        status.session_id        = snapshot->communication.upper_session.session_id;
        status.received_sequence = snapshot->communication.upper_session.received_sequence;
        status.applied_sequence  = snapshot->communication.upper_session.applied_sequence;
        status.reject_reason     = snapshot->communication.upper_session.reject_reason;
        status.ack_flags         = snapshot->communication.upper_session.ack_flags;
    }

    if (link == COMMUNICATION_LINK_ESP12F)
    {
        if (snapshot->communication.esp12f.checksum_errors > 0U)
        {
            status.comm_health_flags |= UPPER_COMM_HEALTH_ESP_CRC;
        }
        if (snapshot->communication.esp12f.rx_overflows > 0U)
        {
            status.comm_health_flags |= UPPER_COMM_HEALTH_ESP_TIMEOUT;
        }
        if (snapshot->communication.esp12f.tx_drops > 0U)
        {
            status.comm_health_flags |= UPPER_COMM_HEALTH_ESP_TX_DROP;
        }
    }
    else
    {
        if (snapshot->communication.upper.checksum_errors > 0U)
        {
            status.comm_health_flags |= UPPER_COMM_HEALTH_CRC_ERR;
        }
        if (snapshot->communication.upper.timeout_resets > 0U || snapshot->communication.upper.rx_overflows > 0U)
        {
            status.comm_health_flags |= UPPER_COMM_HEALTH_TIMEOUT;
        }
        if (snapshot->communication.upper.tx_drops > 0U)
        {
            status.comm_health_flags |= UPPER_COMM_HEALTH_TX_DROP;
        }
    }

    payload_len = UpperProtocol_BuildStatusPayload(&status, payload, sizeof(payload));
    return UpperProtocol_BuildFrame(UPPER_CMD_STATUS, payload, payload_len, out, out_len);
}

uint16_t TelemetryEncoder_BuildDiagnostic(const communication_publish_model_t *snapshot, uint8_t *out, uint16_t out_len)
{
    upper_diagnostic_payload_t diagnostic = {0};
    uint8_t                    payload[ROBOT_LINK_PROTOCOL_DIAGNOSTIC_PAYLOAD_LEN];
    uint8_t                    payload_len;

    if (snapshot == 0 || out == 0)
    {
        return 0U;
    }
    diagnostic.post_done                = snapshot->post.done;
    diagnostic.imu_status_flags         = TelemetryEncoder_ImuFlags(&snapshot->imu);
    diagnostic.post_error_flags         = snapshot->post.error_flags;
    diagnostic.adc_invalid_reason_flags = snapshot->current.invalid_reason_flags;
    diagnostic.task_timeout_mask        = snapshot->safety.task_timeout_mask;
    diagnostic.imu_quality_flags        = TelemetryEncoder_MapImuQuality(&snapshot->imu);
    diagnostic.reset_reason_flags       = snapshot->control.reset_reason_flags;
    diagnostic.uptime_ms                = snapshot->timestamp_ms;
    payload_len                         = UpperProtocol_BuildDiagnosticPayload(&diagnostic, payload, sizeof(payload));
    return UpperProtocol_BuildFrame(UPPER_CMD_DIAGNOSTIC, payload, payload_len, out, out_len);
}

uint16_t TelemetryEncoder_BuildImu(const communication_publish_model_t *snapshot, uint8_t *out, uint16_t out_len)
{
    upper_imu_status_payload_t imu = {0};
    uint8_t                    payload[ROBOT_LINK_PROTOCOL_IMU_STATUS_PAYLOAD_LEN];
    uint8_t                    payload_len;

    if (snapshot == 0 || out == 0 || snapshot->imu.online == 0U)
    {
        return 0U;
    }
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        imu.accel_g[axis]            = snapshot->imu.accel_g[axis];
        imu.gyro_corrected_dps[axis] = snapshot->imu.gyro_dps[axis];
        imu.euler_deg[axis]          = (axis == 0U)   ? snapshot->imu.roll_deg
                                       : (axis == 1U) ? snapshot->imu.pitch_deg
                                                      : snapshot->imu.yaw_deg;
    }
    for (uint8_t index = 0U; index < 4U; ++index)
    {
        imu.quaternion[index] = snapshot->imu.quaternion[index];
    }
    imu.timestamp_ms  = snapshot->imu.timestamp_ms;
    imu.sensor_time   = snapshot->imu.sensor_time;
    imu.sample_count  = snapshot->imu.sample_count;
    imu.quality_flags = TelemetryEncoder_MapImuQuality(&snapshot->imu);
    for (uint8_t index = 0U; index < 7U; ++index)
    {
        imu.quality_counters[index] = snapshot->imu.quality_counters[index];
    }
    imu.status_flags  = TelemetryEncoder_ImuFlags(&snapshot->imu);
    imu.temperature_c = (int8_t)((snapshot->imu.temperature_c >= 0.0f) ? (snapshot->imu.temperature_c + 0.5f)
                                                                       : (snapshot->imu.temperature_c - 0.5f));
    payload_len       = UpperProtocol_BuildImuStatusPayload(&imu, payload, sizeof(payload));
    return UpperProtocol_BuildFrame(UPPER_CMD_IMU_STATUS, payload, payload_len, out, out_len);
}
