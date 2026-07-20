#ifndef ROBOT_LINK_PROTOCOL_H
#define ROBOT_LINK_PROTOCOL_H

#include "communication_protocol_types.h"
#include "communication_types.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define ROBOT_LINK_PROTOCOL_HEAD_0                    0xA5U
#define ROBOT_LINK_PROTOCOL_HEAD_1                    0x5AU
#define ROBOT_LINK_PROTOCOL_VERSION                   COMMUNICATION_PROTOCOL_VERSION
#define ROBOT_LINK_PROTOCOL_MAX_PAYLOAD               COMMUNICATION_MAX_PAYLOAD_LENGTH
#define ROBOT_LINK_PROTOCOL_MAX_FRAME                 (ROBOT_LINK_PROTOCOL_MAX_PAYLOAD + 5U)
#define ROBOT_LINK_PROTOCOL_CMD_LEN(payload_len)      ((uint8_t)(1U + (payload_len)))
#define ROBOT_LINK_PROTOCOL_STATUS_PAYLOAD_LEN        COMMUNICATION_STATUS_PAYLOAD_LENGTH
#define ROBOT_LINK_PROTOCOL_VELOCITY_PAYLOAD_LEN      COMMUNICATION_SET_VELOCITY_PAYLOAD_LENGTH
#define ROBOT_LINK_PROTOCOL_ESTOP_PAYLOAD_LEN         COMMUNICATION_ESTOP_PAYLOAD_LENGTH
#define ROBOT_LINK_PROTOCOL_LINE_CTRL_PAYLOAD_LEN     COMMUNICATION_LINE_CTRL_PAYLOAD_LENGTH
#define ROBOT_LINK_PROTOCOL_CLEAR_FAULT_PAYLOAD_LEN   COMMUNICATION_CLEAR_FAULT_PAYLOAD_LENGTH
#define ROBOT_LINK_PROTOCOL_GET_INFO_PAYLOAD_LEN      COMMUNICATION_GET_INFO_PAYLOAD_LENGTH
#define ROBOT_LINK_PROTOCOL_HELLO_PAYLOAD_LEN         COMMUNICATION_HELLO_PAYLOAD_LENGTH
#define ROBOT_LINK_PROTOCOL_IMU_STATUS_PAYLOAD_LEN    COMMUNICATION_IMU_STATUS_PAYLOAD_LENGTH
#define ROBOT_LINK_PROTOCOL_DIAGNOSTIC_PAYLOAD_LEN    COMMUNICATION_DIAGNOSTIC_PAYLOAD_LENGTH
#define ROBOT_LINK_PROTOCOL_DIAGNOSTIC_SCHEMA_VERSION COMMUNICATION_DIAGNOSTIC_SCHEMA_VERSION
#define ROBOT_LINK_PROTOCOL_MOTOR_COUNT               4U

#define UPPER_STATUS_FLAG_ESTOP                       (1U << 0)
#define UPPER_STATUS_FLAG_FAULT_STOP                  (1U << 1)
#define UPPER_STATUS_FLAG_LINE_ENABLED                (1U << 2)
#define UPPER_STATUS_FLAG_SPEED_VALID_ALL             (1U << 3)

/* 通信健康标志位 */
#define UPPER_COMM_HEALTH_CRC_ERR                     (1U << 0)
#define UPPER_COMM_HEALTH_TIMEOUT                     (1U << 1)
#define UPPER_COMM_HEALTH_TX_DROP                     (1U << 2)
#define UPPER_COMM_HEALTH_ESP_CRC                     (1U << 3)
#define UPPER_COMM_HEALTH_ESP_TIMEOUT                 (1U << 4)
#define UPPER_COMM_HEALTH_ESP_TX_DROP                 (1U << 5)

/* IMU 状态标志位 */
#define UPPER_IMU_FLAG_ONLINE                         (1U << 0)
#define UPPER_IMU_FLAG_CALIBRATED                     (1U << 1)
#define UPPER_IMU_FLAG_ERROR                          (1U << 2)
#define UPPER_IMU_FLAG_SENSOR_TIME                    (1U << 3)

    typedef enum
    {
        UPPER_CMD_SET_VELOCITY = 0x01,
        UPPER_CMD_ESTOP        = 0x02,
        UPPER_CMD_LINE_CTRL    = 0x03,
        UPPER_CMD_CLEAR_FAULT  = 0x04,
        UPPER_CMD_GET_INFO     = 0x05,
        UPPER_CMD_HELLO        = 0x80,
        UPPER_CMD_STATUS       = 0x81,
        UPPER_CMD_DIAGNOSTIC   = 0x82,
        UPPER_CMD_IMU_STATUS   = 0x83
    } robot_link_protocol_cmd_t;

    typedef struct
    {
        float    linear_x;
        float    angular_z;
        uint8_t  enable;
        uint8_t  mode;
        uint64_t session_id;
        uint32_t sequence;
    } upper_velocity_payload_t;

    typedef struct
    {
        float    battery_voltage;
        float    motor_speed_mps[ROBOT_LINK_PROTOCOL_MOTOR_COUNT];
        int32_t  encoder_count[ROBOT_LINK_PROTOCOL_MOTOR_COUNT];
        float    motor_current_a[ROBOT_LINK_PROTOCOL_MOTOR_COUNT];
        float    motor_target_mps[ROBOT_LINK_PROTOCOL_MOTOR_COUNT];
        int16_t  motor_output_permille[ROBOT_LINK_PROTOCOL_MOTOR_COUNT];
        uint32_t error_flags;
        uint32_t latched_error_flags;
        uint8_t  status_flags;
        uint8_t  control_source;
        uint8_t  motor_enabled_mask;
        uint8_t  motor_speed_valid_mask;
        uint8_t  encoder_anomaly_mask;
        uint8_t  comm_health_flags;
        uint32_t status_sequence;
        uint32_t timestamp_ms;
        uint64_t session_id;
        uint32_t received_sequence;
        uint32_t applied_sequence;
        uint8_t  reject_reason;
        uint8_t  side_consistency_flags;
        uint8_t  ack_flags;
    } upper_status_payload_t;

    typedef struct
    {
        float    accel_g[3];
        float    gyro_corrected_dps[3];
        float    euler_deg[3];
        float    quaternion[4];
        uint32_t timestamp_ms;
        uint32_t sensor_time;
        uint32_t sample_count;
        uint32_t quality_flags;
        uint32_t quality_counters[7];
        uint8_t  status_flags;
        int8_t   temperature_c;
    } upper_imu_status_payload_t;

    typedef struct
    {
        uint8_t  post_done;
        uint8_t  imu_status_flags;
        uint32_t post_error_flags;
        uint32_t adc_invalid_reason_flags;
        uint16_t task_timeout_mask;
        uint32_t imu_quality_flags;
        uint32_t reset_reason_flags;
        uint32_t uptime_ms;
    } upper_diagnostic_payload_t;

    typedef struct
    {
        uint8_t                           schema_version;
        communication_firmware_identity_t identity;
        uint32_t                          parameter_crc32;
    } upper_hello_payload_t;

    uint8_t UpperProtocol_Checksum8(const uint8_t *data, uint16_t length);
    uint8_t UpperProtocol_RemoteEstopSetRequested(const uint8_t *payload, uint8_t payload_len);
    uint16_t
    UpperProtocol_BuildFrame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len, uint8_t *out, uint16_t out_len);
    uint8_t
    UpperProtocol_ParseVelocityPayload(const uint8_t *payload, uint8_t payload_len, upper_velocity_payload_t *velocity);
    uint8_t UpperProtocol_ParseVersionedFlag(const uint8_t *payload,
                                             uint8_t        payload_len,
                                             uint8_t        expected_payload_len,
                                             uint8_t       *value);
    uint8_t UpperProtocol_ParseVersionOnly(const uint8_t *payload, uint8_t payload_len, uint8_t expected_payload_len);
    uint8_t UpperProtocol_BuildHelloPayload(const upper_hello_payload_t *hello, uint8_t *out, uint8_t out_len);
    uint8_t UpperProtocol_BuildStatusPayload(const upper_status_payload_t *status, uint8_t *out, uint8_t out_len);
    uint8_t UpperProtocol_BuildImuStatusPayload(const upper_imu_status_payload_t *imu, uint8_t *out, uint8_t out_len);
    uint8_t
    UpperProtocol_BuildDiagnosticPayload(const upper_diagnostic_payload_t *diagnostic, uint8_t *out, uint8_t out_len);

#ifdef __cplusplus
}
#endif

#endif
