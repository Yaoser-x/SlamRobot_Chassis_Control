#include "remote_command_dispatcher.h"

#include "command_management_service.h"
#include "communication_operation_mailbox.h"
#include "communication_session_tracker.h"
#include "robot_link_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static command_velocity_t last_command;
static uint8_t            command_count;
static uint8_t            disable_count;
static uint8_t            refresh_count;

uint32_t PlatformCritical_Enter(void)
{
    return 0UL;
}

void PlatformCritical_Exit(uint32_t state)
{
    (void)state;
}

command_result_t CommandManagement_Set(const command_velocity_t *command)
{
    last_command = *command;
    command_count++;
    return COMMAND_RESULT_ACCEPTED;
}

command_result_t CommandManagement_DisableRemoteSource(command_source_t source)
{
    (void)source;
    disable_count++;
    return COMMAND_RESULT_ACCEPTED;
}

uint8_t CommandManagement_RefreshSource(command_source_t source, uint32_t now_ms)
{
    (void)source;
    (void)now_ms;
    refresh_count++;
    return 1U;
}

uint8_t CommandManagement_IsMotionGateOpen(void)
{
    return 1U;
}

static void WriteU32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
    out[2] = (uint8_t)(value >> 16);
    out[3] = (uint8_t)(value >> 24);
}

static void WriteU64(uint8_t *out, uint64_t value)
{
    WriteU32(out, (uint32_t)value);
    WriteU32(&out[4], (uint32_t)(value >> 32));
}

static protocol_frame_t
VelocityFrame(float linear_x, float angular_z, uint8_t enable, uint64_t session_id, uint32_t sequence)
{
    protocol_frame_t frame = {
        .cmd         = UPPER_CMD_SET_VELOCITY,
        .payload_len = ROBOT_LINK_PROTOCOL_VELOCITY_PAYLOAD_LEN,
    };

    frame.payload[0] = ROBOT_LINK_PROTOCOL_VERSION;
    (void)memcpy(&frame.payload[1], &linear_x, sizeof(linear_x));
    (void)memcpy(&frame.payload[5], &angular_z, sizeof(angular_z));
    frame.payload[9]  = enable;
    frame.payload[10] = 2U;
    WriteU64(&frame.payload[11], session_id);
    WriteU32(&frame.payload[19], sequence);
    return frame;
}

int main(void)
{
    protocol_frame_t                  frame;
    communication_operation_request_t operation;

    CommunicationSessionTracker_Init();
    CommunicationOperationMailbox_ResetLink(COMMUNICATION_LINK_UPPER);
    CommunicationOperationMailbox_ResetLink(COMMUNICATION_LINK_ESP12F);
    frame = VelocityFrame(0.0f, 0.0f, 0U, 0x1122334455667788ULL, 1U);
    (void)RemoteCommandDispatcher_Handle(COMMUNICATION_LINK_UPPER, &frame, 100U);
    assert(disable_count == 1U);

    frame = VelocityFrame(0.5f, -0.25f, 1U, 0x1122334455667788ULL, 2U);
    (void)RemoteCommandDispatcher_Handle(COMMUNICATION_LINK_UPPER, &frame, 123U);
    assert(command_count == 1U);
    assert(last_command.source == COMMAND_SOURCE_HOST);
    assert(last_command.timestamp_ms == 123U);

    (void)RemoteCommandDispatcher_Handle(COMMUNICATION_LINK_UPPER, &frame, 150U);
    assert(command_count == 1U);
    assert(refresh_count == 1U);

    frame = (protocol_frame_t){
        .cmd         = UPPER_CMD_ESTOP,
        .payload_len = ROBOT_LINK_PROTOCOL_ESTOP_PAYLOAD_LEN,
        .payload     = {ROBOT_LINK_PROTOCOL_VERSION, 1U},
    };
    (void)RemoteCommandDispatcher_Handle(COMMUNICATION_LINK_UPPER, &frame, 0U);
    assert(CommunicationOperationMailbox_Take(COMMUNICATION_LINK_UPPER, 1U, &operation) != 0U);
    assert(operation.kind == COMMUNICATION_OPERATION_ESTOP && operation.value == 1U);

    frame = (protocol_frame_t){
        .cmd         = UPPER_CMD_LINE_CTRL,
        .payload_len = ROBOT_LINK_PROTOCOL_LINE_CTRL_PAYLOAD_LEN,
        .payload     = {ROBOT_LINK_PROTOCOL_VERSION, 1U},
    };
    (void)RemoteCommandDispatcher_Handle(COMMUNICATION_LINK_UPPER, &frame, 0U);
    assert(CommunicationOperationMailbox_Take(COMMUNICATION_LINK_UPPER, 1U, &operation) != 0U);
    assert(operation.kind == COMMUNICATION_OPERATION_LINE_CTRL && operation.value == 1U);

    frame = (protocol_frame_t){
        .cmd         = UPPER_CMD_CLEAR_FAULT,
        .payload_len = ROBOT_LINK_PROTOCOL_CLEAR_FAULT_PAYLOAD_LEN,
        .payload     = {ROBOT_LINK_PROTOCOL_VERSION},
    };
    (void)RemoteCommandDispatcher_Handle(COMMUNICATION_LINK_ESP12F, &frame, 0U);
    assert(CommunicationOperationMailbox_Take(COMMUNICATION_LINK_ESP12F, 1U, &operation) != 0U);
    assert(operation.kind == COMMUNICATION_OPERATION_CLEAR_FAULT);

    frame = (protocol_frame_t){
        .cmd         = UPPER_CMD_GET_INFO,
        .payload_len = ROBOT_LINK_PROTOCOL_GET_INFO_PAYLOAD_LEN,
        .payload     = {ROBOT_LINK_PROTOCOL_VERSION},
    };
    assert(RemoteCommandDispatcher_Handle(COMMUNICATION_LINK_UPPER, &frame, 0U) == REMOTE_ACTION_REQUEST_INFO);

    puts("PASS: Upper Protocol v3 command dispatcher");
    return 0;
}
