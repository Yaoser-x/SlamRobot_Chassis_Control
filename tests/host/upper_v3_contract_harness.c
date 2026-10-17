/* Host I/O adapter around the production v3 parser, dispatcher and command owner.
 * Only parameter snapshots and the hardware-free critical section are injected. */
#include "command_management_service.h"
#include "communication_session_tracker.h"
#include "parameter_management_service.h"
#include "remote_command_dispatcher.h"
#include "robot_link_protocol.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static frame_parser_t                    parsers[COMMUNICATION_LINK_COUNT];
static uint32_t                          status_sequence;
static communication_firmware_identity_t identity = {
    .hardware_revision = 0x00020000UL,
    .capabilities      = COMMUNICATION_REQUIRED_CAPABILITIES,
};
static uint32_t parameter_crc32 = 0x12345678UL;

uint32_t ParameterManagement_GetSnapshot(param_model_t *params)
{
    *params                 = (param_model_t){0};
    params->max_linear_mps  = 0.5f;
    params->max_angular_rps = 10.0f;
    params->wheel_radius_m  = 0.035f;
    params->track_width_m   = 0.176f;
    return 1UL;
}

static void PrintHex(const uint8_t *bytes, uint16_t length)
{
    for (uint16_t i = 0U; i < length; ++i)
    {
        (void)printf("%02x", bytes[i]);
    }
}

static void Emit(communication_link_t link, uint32_t now_ms, uint8_t hello_requested, uint32_t received_frames)
{
    command_management_status_t      command;
    command_velocity_t               active;
    communication_session_snapshot_t session;
    upper_status_payload_t           status = {0};
    uint8_t                          payload[ROBOT_LINK_PROTOCOL_MAX_PAYLOAD];
    uint8_t                          frame[ROBOT_LINK_PROTOCOL_MAX_FRAME];
    uint8_t                          has_active = CommandManagement_GetActiveSnapshot(now_ms, &active, &command);
    uint8_t                          payload_length;
    uint16_t                         frame_length;

    CommunicationSessionTracker_GetSnapshot(link, &session);
    status.status_sequence        = ++status_sequence;
    status.timestamp_ms           = now_ms;
    status.motor_enabled_mask     = 0x06U;
    status.motor_speed_valid_mask = 0x06U;
    status.battery_voltage        = 12.0f;
    status.control_source         = (uint8_t)command.active_source;
    status.status_flags           = (command.motion_allowed != 0U) ? 0U : UPPER_STATUS_FLAG_FAULT_STOP;
    status.session_id             = session.session_id;
    status.received_sequence      = session.received_sequence;
    status.applied_sequence       = session.applied_sequence;
    status.reject_reason          = session.reject_reason;
    status.ack_flags              = session.ack_flags;
    payload_length                = UpperProtocol_BuildStatusPayload(&status, payload, sizeof(payload));
    frame_length = UpperProtocol_BuildFrame(UPPER_CMD_STATUS, payload, payload_length, frame, sizeof(frame));
    (void)printf("{\"active\":%u,\"source\":%u,\"rearm\":%u,\"frames\":%" PRIu32 ",\"status\":\"",
                 has_active,
                 (unsigned)command.active_source,
                 command.rearm_required_mask,
                 received_frames);
    PrintHex(frame, frame_length);
    (void)printf("\",\"hello\":\"");
    if (hello_requested != 0U)
    {
        upper_hello_payload_t hello = {.schema_version = 1U, .identity = identity, .parameter_crc32 = parameter_crc32};
        payload_length              = UpperProtocol_BuildHelloPayload(&hello, payload, sizeof(payload));
        frame_length = UpperProtocol_BuildFrame(UPPER_CMD_HELLO, payload, payload_length, frame, sizeof(frame));
        PrintHex(frame, frame_length);
    }
    (void)puts("\"}");
    (void)fflush(stdout);
}

int main(void)
{
    const command_management_config_t config = {
        .host_timeout_ms        = 200U,
        .ps2_timeout_ms         = 500U,
        .esp12f_timeout_ms      = 500U,
        .line_timeout_ms        = 50U,
        .debug_timeout_ms       = 2000U,
        .remote_max_lifetime_ms = 2000U,
    };
    char     line[1024];
    unsigned link;
    uint32_t now_ms;
    (void)memset(identity.git_commit, 0x11, sizeof(identity.git_commit));
    if (CommandManagement_Init(&config) == 0U)
    {
        return 2;
    }
    CommandManagement_SetMotionGate(1U, 1U);
    CommunicationSessionTracker_Init();
    for (unsigned i = 0U; i < COMMUNICATION_LINK_COUNT; ++i)
    {
        FrameParser_Init(&parsers[i]);
    }
    while (fgets(line, sizeof(line), stdin) != 0)
    {
        char     hex[600];
        unsigned allowed;
        unsigned mask;
        unsigned rearm;
        unsigned commit_byte;
        uint32_t generation;
        uint32_t revision;
        uint32_t crc;
        uint32_t sample;
        uint32_t sensor_time;
        uint32_t quality;
        if (sscanf(line, "wire %u %" SCNu32 " %599s", &link, &now_ms, hex) == 3 && link < COMMUNICATION_LINK_COUNT)
        {
            uint32_t received_frames = 0U;
            uint8_t  hello_requested = 0U;
            size_t   length          = strlen(hex);
            for (size_t i = 0U; i + 1U < length; i += 2U)
            {
                unsigned         byte;
                protocol_frame_t received;
                if (sscanf(&hex[i], "%2x", &byte) != 1)
                {
                    return 3;
                }
                if (FrameParser_Push(&parsers[link], (uint8_t)byte, &received) == FRAME_PARSER_FRAME)
                {
                    received_frames++;
                    if (RemoteCommandDispatcher_Handle((communication_link_t)link, &received, now_ms)
                        == REMOTE_ACTION_REQUEST_INFO)
                    {
                        hello_requested = 1U;
                    }
                }
            }
            Emit((communication_link_t)link, now_ms, hello_requested, received_frames);
        }
        else if (sscanf(line, "poll %u %" SCNu32, &link, &now_ms) == 2 && link < COMMUNICATION_LINK_COUNT)
        {
            Emit((communication_link_t)link, now_ms, 0U, 0U);
        }
        else if (sscanf(line, "gate %u %" SCNu32 " %" SCNu32, &allowed, &generation, &now_ms) == 3)
        {
            CommandManagement_SetMotionGate((uint8_t)allowed, generation);
            Emit(COMMUNICATION_LINK_UPPER, now_ms, 0U, 0U);
        }
        else if (sscanf(line, "policy %u %u %" SCNu32 " %" SCNu32, &mask, &rearm, &generation, &now_ms) == 4)
        {
            if (CommandManagement_ApplySourcePolicy((uint8_t)mask, (uint8_t)rearm, 0U, generation) == 0U)
            {
                return 4;
            }
            Emit(COMMUNICATION_LINK_UPPER, now_ms, 0U, 0U);
        }
        else if (sscanf(line, "identity %u %" SCNu32 " %" SCNu32 " %" SCNu32, &commit_byte, &revision, &crc, &now_ms)
                 == 4)
        {
            (void)memset(identity.git_commit, (int)commit_byte, sizeof(identity.git_commit));
            identity.hardware_revision = revision;
            parameter_crc32            = crc;
            Emit(COMMUNICATION_LINK_UPPER, now_ms, 1U, 0U);
        }
        else if (sscanf(line,
                        "imu %" SCNu32 " %" SCNu32 " %" SCNu32 " %" SCNu32,
                        &sample,
                        &sensor_time,
                        &quality,
                        &now_ms)
                 == 4)
        {
            upper_imu_status_payload_t imu = {
                .gyro_corrected_dps = {0.0f, 0.0f, 5.0f},
                .quaternion         = {1.0f, 0.0f, 0.0f, 0.0f},
                .sample_count       = sample,
                .sensor_time        = sensor_time & 0x00FFFFFFUL,
                .timestamp_ms       = now_ms,
                .quality_flags      = quality,
                .status_flags       = UPPER_IMU_FLAG_ONLINE | UPPER_IMU_FLAG_CALIBRATED,
            };
            uint8_t  payload[ROBOT_LINK_PROTOCOL_MAX_PAYLOAD];
            uint8_t  frame[ROBOT_LINK_PROTOCOL_MAX_FRAME];
            uint8_t  payload_length = UpperProtocol_BuildImuStatusPayload(&imu, payload, sizeof(payload));
            uint16_t frame_length =
                UpperProtocol_BuildFrame(UPPER_CMD_IMU_STATUS, payload, payload_length, frame, sizeof(frame));
            (void)printf("{\"imu\":\"");
            PrintHex(frame, frame_length);
            (void)puts("\"}");
            (void)fflush(stdout);
        }
        else
        {
            return 5;
        }
    }
    return 0;
}
