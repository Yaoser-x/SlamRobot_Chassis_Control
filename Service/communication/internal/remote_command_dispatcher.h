#ifndef COMMUNICATION_COMMAND_ROUTER_H
#define COMMUNICATION_COMMAND_ROUTER_H

#include <stdint.h>

#include "communication_types.h"
#include "frame_stream_parser.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        REMOTE_ACTION_NONE = 0,
        REMOTE_ACTION_REQUEST_INFO
    } remote_command_action_t;

    /** Route one decoded frame and return a link-local transport action. */
    remote_command_action_t
    RemoteCommandDispatcher_Handle(communication_link_t link, const protocol_frame_t *frame, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
