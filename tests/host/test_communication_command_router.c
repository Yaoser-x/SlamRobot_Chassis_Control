#include "communication_command_router.h"

#include "control_service.h"
#include "upper_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static chassis_cmd_t last_command;
static uint8_t       command_count;
static uint8_t       emergency_stop;
static uint8_t       line_enabled;
static uint32_t      cleared_fault_mask;

control_command_result_t ControlService_SetCommand(const chassis_cmd_t *command)
{
    last_command = *command;
    command_count++;
    return CONTROL_COMMAND_ACCEPTED;
}

void ControlService_SetEmergencyStop(uint8_t enabled)
{
    emergency_stop = enabled;
}

uint8_t ControlService_IsEmergencyStop(void)
{
    return emergency_stop;
}

void LineControlService_Enable(uint8_t enabled)
{
    line_enabled = enabled;
}

void SafetyService_ClearLatchedFaults(uint32_t mask)
{
    cleared_fault_mask = mask;
}

static protocol_frame_t VelocityFrame(float linear_x, float angular_z)
{
    protocol_frame_t frame = {.cmd = UPPER_CMD_SET_VELOCITY, .payload_len = UPPER_PROTOCOL_VELOCITY_PAYLOAD_LEN};

    (void)memcpy(&frame.payload[0], &linear_x, sizeof(linear_x));
    (void)memcpy(&frame.payload[4], &angular_z, sizeof(angular_z));
    frame.payload[8] = 1U;
    frame.payload[9] = 7U;
    return frame;
}

int main(void)
{
    protocol_frame_t frame = VelocityFrame(0.5f, -0.25f);

    CommunicationCommandRouter_Handle(COMMUNICATION_LINK_UPPER, &frame, 123U);
    assert(command_count == 1U);
    assert(last_command.source == CONTROL_SOURCE_UPPER);
    assert(last_command.timestamp_ms == 123U);
    assert(last_command.linear_x == 0.5f);
    assert(last_command.angular_z == -0.25f);

    CommunicationCommandRouter_Handle(COMMUNICATION_LINK_ESP12F, &frame, 456U);
    assert(command_count == 2U);
    assert(last_command.source == CONTROL_SOURCE_ESP12F);
    assert(last_command.timestamp_ms == 456U);

    frame            = (protocol_frame_t){.cmd = UPPER_CMD_ESTOP, .payload_len = UPPER_PROTOCOL_ESTOP_PAYLOAD_LEN};
    frame.payload[0] = 1U;
    CommunicationCommandRouter_Handle(COMMUNICATION_LINK_UPPER, &frame, 0U);
    assert(emergency_stop == 1U);

    frame = (protocol_frame_t){.cmd = UPPER_CMD_LINE_CTRL, .payload_len = UPPER_PROTOCOL_LINE_CTRL_PAYLOAD_LEN};
    frame.payload[0] = 1U;
    CommunicationCommandRouter_Handle(COMMUNICATION_LINK_UPPER, &frame, 0U);
    assert(line_enabled == 1U);

    frame = (protocol_frame_t){.cmd = UPPER_CMD_CLEAR_FAULT, .payload_len = UPPER_PROTOCOL_CLEAR_FAULT_PAYLOAD_LEN};
    CommunicationCommandRouter_Handle(COMMUNICATION_LINK_UPPER, &frame, 0U);
    assert(cleared_fault_mask == 0U);
    emergency_stop = 0U;
    CommunicationCommandRouter_Handle(COMMUNICATION_LINK_ESP12F, &frame, 0U);
    assert(cleared_fault_mask == 0xFFFFFFFFUL);

    puts("PASS: communication command router");
    return 0;
}
