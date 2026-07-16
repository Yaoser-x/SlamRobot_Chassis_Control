#ifndef COMMUNICATION_COMMAND_ROUTER_H
#define COMMUNICATION_COMMAND_ROUTER_H

#include <stdint.h>

#include "communication_types.h"
#include "frame_stream_parser.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** Route one decoded protocol frame to the shared control services. */
    void RemoteCommandDispatcher_Handle(communication_link_t link, const protocol_frame_t *frame, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
