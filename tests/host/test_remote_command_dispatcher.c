#include "remote_command_dispatcher.h"

#include "command_management_service.h"
#include "communication_session_tracker.h"
#include "robot_link_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static command_velocity_t last_command;
static uint8_t            command_count;
static uint8_t            clear_count;
static uint8_t            refresh_count;
static uint8_t            emergency_stop;
static uint8_t            line_enabled;
static uint32_t           cleared_fault_mask;

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

void CommandManagement_ClearSource(command_source_t source)
{
    (void)source;
    clear_count++;
}

uint8_t CommandManagement_RefreshSource(command_source_t source, uint32_t now_ms)
{
    (void)source;
    (void)now_ms;
    refresh_count++;
    return 1U;
}

uint8_t SafetyManagement_IsMotionAllowed(void)
{
    return 1U;
}

void SafetyManagement_SetEmergencyStop(uint8_t enabled)
{
    emergency_stop = enabled;
}

uint8_t SafetyManagement_IsEmergencyStop(void)
{
    return emergency_stop;
}

void LineFollowing_Enable(uint8_t enabled)
{
    line_enabled = enabled;
}

void SafetyManagement_ClearLatchedFaults(uint32_t mask)
{
    cleared_fault_mask = mask;
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
    protocol_frame_t frame;

    CommunicationSessionTracker_Init();
    frame = VelocityFrame(0.0f, 0.0f, 0U, 0x1122334455667788ULL, 1U);
    (void)RemoteCommandDispatcher_Handle(COMMUNICATION_LINK_UPPER, &frame, 100U);
    assert(clear_count == 1U);

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
    assert(emergency_stop == 1U);

    frame = (protocol_frame_t){
        .cmd         = UPPER_CMD_LINE_CTRL,
        .payload_len = ROBOT_LINK_PROTOCOL_LINE_CTRL_PAYLOAD_LEN,
        .payload     = {ROBOT_LINK_PROTOCOL_VERSION, 1U},
    };
    (void)RemoteCommandDispatcher_Handle(COMMUNICATION_LINK_UPPER, &frame, 0U);
    assert(line_enabled == 1U);

    emergency_stop = 0U;
    frame          = (protocol_frame_t){
                 .cmd         = UPPER_CMD_CLEAR_FAULT,
                 .payload_len = ROBOT_LINK_PROTOCOL_CLEAR_FAULT_PAYLOAD_LEN,
                 .payload     = {ROBOT_LINK_PROTOCOL_VERSION},
    };
    (void)RemoteCommandDispatcher_Handle(COMMUNICATION_LINK_ESP12F, &frame, 0U);
    assert(cleared_fault_mask == 0xFFFFFFFFUL);

    frame = (protocol_frame_t){
        .cmd         = UPPER_CMD_GET_INFO,
        .payload_len = ROBOT_LINK_PROTOCOL_GET_INFO_PAYLOAD_LEN,
        .payload     = {ROBOT_LINK_PROTOCOL_VERSION},
    };
    assert(RemoteCommandDispatcher_Handle(COMMUNICATION_LINK_UPPER, &frame, 0U) == REMOTE_ACTION_REQUEST_INFO);

    puts("PASS: Upper Protocol v3 command dispatcher");
    return 0;
}
