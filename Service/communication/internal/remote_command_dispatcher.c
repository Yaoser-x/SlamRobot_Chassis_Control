#include "remote_command_dispatcher.h"

#include "command_management_service.h"
#include "line_following_service.h"
#include "safety_management_service.h"
#include "robot_link_protocol.h"

void RemoteCommandDispatcher_Handle(communication_link_t link, const protocol_frame_t *frame, uint32_t now_ms)
{
    command_source_t source;

    if (frame == 0)
    {
        return;
    }
    source = (link == COMMUNICATION_LINK_ESP12F) ? COMMAND_SOURCE_ESP12F : COMMAND_SOURCE_HOST;

    if (frame->cmd == UPPER_CMD_SET_VELOCITY)
    {
        upper_velocity_payload_t velocity;
        if (UpperProtocol_ParseVelocityPayload(frame->payload, frame->payload_len, &velocity) != 0U)
        {
            command_velocity_t command = {
                .linear_x     = velocity.linear_x,
                .angular_z    = velocity.angular_z,
                .enable       = velocity.enable,
                .source       = source,
                .timestamp_ms = now_ms,
            };
            (void)velocity.mode;
            (void)CommandManagement_Set(&command);
        }
    }
    else if (frame->cmd == UPPER_CMD_ESTOP && frame->payload_len == ROBOT_LINK_PROTOCOL_ESTOP_PAYLOAD_LEN)
    {
        if (UpperProtocol_RemoteEstopSetRequested(frame->payload, frame->payload_len) != 0U)
        {
            SafetyManagement_SetEmergencyStop(1U);
        }
    }
    else if (frame->cmd == UPPER_CMD_LINE_CTRL && frame->payload_len == ROBOT_LINK_PROTOCOL_LINE_CTRL_PAYLOAD_LEN)
    {
        LineFollowing_Enable((frame->payload[0] != 0U) ? 1U : 0U);
    }
    else if (frame->cmd == UPPER_CMD_CLEAR_FAULT && frame->payload_len == ROBOT_LINK_PROTOCOL_CLEAR_FAULT_PAYLOAD_LEN
             && SafetyManagement_IsEmergencyStop() == 0U)
    {
        SafetyManagement_ClearLatchedFaults(0xFFFFFFFFUL);
    }
}
