#include "communication_command_router.h"

#include "control_config.h"
#include "control_service.h"
#include "line_control_service.h"
#include "safety_service.h"
#include "upper_protocol.h"

void CommunicationCommandRouter_Handle(communication_link_t link, const protocol_frame_t *frame, uint32_t now_ms)
{
    uint8_t source;

    if (frame == 0)
    {
        return;
    }
    source = (link == COMMUNICATION_LINK_ESP12F) ? CONTROL_SOURCE_ESP12F : CONTROL_SOURCE_UPPER;

    if (frame->cmd == UPPER_CMD_SET_VELOCITY)
    {
        upper_velocity_payload_t velocity;
        if (UpperProtocol_ParseVelocityPayload(frame->payload, frame->payload_len, &velocity) != 0U)
        {
            chassis_cmd_t chassis_cmd = {
                .linear_x     = velocity.linear_x,
                .angular_z    = velocity.angular_z,
                .enable       = velocity.enable,
                .source       = source,
                .timestamp_ms = now_ms,
            };
            (void)velocity.mode;
            (void)ControlService_SetCommand(&chassis_cmd);
        }
    }
    else if (frame->cmd == UPPER_CMD_ESTOP && frame->payload_len == UPPER_PROTOCOL_ESTOP_PAYLOAD_LEN)
    {
        if (UpperProtocol_RemoteEstopSetRequested(frame->payload, frame->payload_len) != 0U)
        {
            ControlService_SetEmergencyStop(1U);
        }
    }
    else if (frame->cmd == UPPER_CMD_LINE_CTRL && frame->payload_len == UPPER_PROTOCOL_LINE_CTRL_PAYLOAD_LEN)
    {
        LineControlService_Enable((frame->payload[0] != 0U) ? 1U : 0U);
    }
    else if (frame->cmd == UPPER_CMD_CLEAR_FAULT && frame->payload_len == UPPER_PROTOCOL_CLEAR_FAULT_PAYLOAD_LEN
             && ControlService_IsEmergencyStop() == 0U)
    {
        SafetyService_ClearLatchedFaults(0xFFFFFFFFUL);
    }
}
