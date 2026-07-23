#include "remote_command_dispatcher.h"

#include "communication_operation_mailbox.h"
#include "communication_session_tracker.h"
#include "command_management_service.h"
#include "robot_link_protocol.h"

static uint8_t RemoteCommandDispatcher_Finite(float value)
{
    return (value == value && value < 3.402823466e+38f && value > -3.402823466e+38f) ? 1U : 0U;
}

static uint8_t RemoteCommandDispatcher_CommandRejectReason(void)
{
    return (CommandManagement_IsMotionGateOpen() == 0U) ? COMMUNICATION_REJECT_FAULT
                                                        : COMMUNICATION_REJECT_SOURCE_NOT_PERMITTED;
}

static void RemoteCommandDispatcher_HandleVelocity(communication_link_t    link,
                                                   command_source_t        source,
                                                   const protocol_frame_t *frame,
                                                   uint32_t                now_ms)
{
    upper_velocity_payload_t         velocity;
    communication_wire_target_t      target;
    communication_session_decision_t decision;
    uint8_t                          applied = 0U;

    if (frame->payload_len != ROBOT_LINK_PROTOCOL_VELOCITY_PAYLOAD_LEN)
    {
        CommunicationSessionTracker_RecordReject(link, COMMUNICATION_REJECT_MALFORMED);
        return;
    }
    if (frame->payload[0] != ROBOT_LINK_PROTOCOL_VERSION)
    {
        CommunicationSessionTracker_RecordReject(link, COMMUNICATION_REJECT_UNSUPPORTED_VERSION);
        return;
    }
    if (UpperProtocol_ParseVelocityPayload(frame->payload, frame->payload_len, &velocity) == 0U)
    {
        CommunicationSessionTracker_RecordReject(link, COMMUNICATION_REJECT_MALFORMED);
        return;
    }
    if (RemoteCommandDispatcher_Finite(velocity.linear_x) == 0U
        || RemoteCommandDispatcher_Finite(velocity.angular_z) == 0U)
    {
        CommunicationSessionTracker_RecordReject(link, COMMUNICATION_REJECT_NONFINITE);
        return;
    }
    if (velocity.enable > 1U || velocity.mode != 2U)
    {
        CommunicationSessionTracker_RecordReject(link, COMMUNICATION_REJECT_INVALID_MODE);
        return;
    }
    target = (communication_wire_target_t){
        .linear_x   = velocity.linear_x,
        .angular_z  = velocity.angular_z,
        .session_id = velocity.session_id,
        .sequence   = velocity.sequence,
        .enable     = velocity.enable,
        .mode       = velocity.mode,
    };
    decision = CommunicationSessionTracker_Evaluate(link, &target, now_ms);
    if (decision == COMMUNICATION_SESSION_DISABLE)
    {
        applied = (CommandManagement_DisableRemoteSource(source) == COMMAND_RESULT_ACCEPTED) ? 1U : 0U;
    }
    else if (decision == COMMUNICATION_SESSION_NEW_COMMAND)
    {
        command_velocity_t command = {
            .linear_x     = velocity.linear_x,
            .angular_z    = velocity.angular_z,
            .enable       = 1U,
            .source       = source,
            .timestamp_ms = now_ms,
        };
        applied = (CommandManagement_Set(&command) == COMMAND_RESULT_ACCEPTED) ? 1U : 0U;
    }
    else if (decision == COMMUNICATION_SESSION_DUPLICATE_KEEPALIVE)
    {
        if (velocity.enable == 0U)
        {
            applied = 1U;
        }
        else
        {
            applied = CommandManagement_RefreshSource(source, now_ms);
        }
    }
    if (decision == COMMUNICATION_SESSION_DISABLE || decision == COMMUNICATION_SESSION_NEW_COMMAND
        || decision == COMMUNICATION_SESSION_DUPLICATE_KEEPALIVE)
    {
        CommunicationSessionTracker_Complete(link,
                                             velocity.sequence,
                                             applied,
                                             (applied != 0U) ? COMMUNICATION_REJECT_NONE
                                                             : RemoteCommandDispatcher_CommandRejectReason());
    }
}

remote_command_action_t
RemoteCommandDispatcher_Handle(communication_link_t link, const protocol_frame_t *frame, uint32_t now_ms)
{
    command_source_t source;

    if (frame == 0)
    {
        return REMOTE_ACTION_NONE;
    }
    source = (link == COMMUNICATION_LINK_ESP12F) ? COMMAND_SOURCE_ESP12F : COMMAND_SOURCE_HOST;

    if (frame->cmd == UPPER_CMD_SET_VELOCITY)
    {
        RemoteCommandDispatcher_HandleVelocity(link, source, frame, now_ms);
    }
    else if (frame->cmd == UPPER_CMD_ESTOP)
    {
        uint8_t requested;
        if (UpperProtocol_ParseVersionedFlag(frame->payload,
                                             frame->payload_len,
                                             ROBOT_LINK_PROTOCOL_ESTOP_PAYLOAD_LEN,
                                             &requested)
                != 0U
            && requested != 0U)
        {
            (void)CommunicationOperationMailbox_Enqueue(link, COMMUNICATION_OPERATION_ESTOP, 1U, now_ms);
        }
    }
    else if (frame->cmd == UPPER_CMD_LINE_CTRL)
    {
        uint8_t enabled;
        if (UpperProtocol_ParseVersionedFlag(frame->payload,
                                             frame->payload_len,
                                             ROBOT_LINK_PROTOCOL_LINE_CTRL_PAYLOAD_LEN,
                                             &enabled)
            != 0U)
        {
            (void)CommunicationOperationMailbox_Enqueue(link,
                                                        COMMUNICATION_OPERATION_LINE_CTRL,
                                                        (enabled != 0U) ? 1U : 0U,
                                                        now_ms);
        }
    }
    else if (frame->cmd == UPPER_CMD_CLEAR_FAULT
             && UpperProtocol_ParseVersionOnly(frame->payload,
                                               frame->payload_len,
                                               ROBOT_LINK_PROTOCOL_CLEAR_FAULT_PAYLOAD_LEN)
                    != 0U)
    {
        (void)CommunicationOperationMailbox_Enqueue(link, COMMUNICATION_OPERATION_CLEAR_FAULT, 1U, now_ms);
    }
    else if (frame->cmd == UPPER_CMD_GET_INFO
             && UpperProtocol_ParseVersionOnly(frame->payload,
                                               frame->payload_len,
                                               ROBOT_LINK_PROTOCOL_GET_INFO_PAYLOAD_LEN)
                    != 0U)
    {
        return REMOTE_ACTION_REQUEST_INFO;
    }
    return REMOTE_ACTION_NONE;
}
