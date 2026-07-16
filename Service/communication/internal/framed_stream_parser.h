#ifndef FRAMED_STREAM_PARSER_H
#define FRAMED_STREAM_PARSER_H

#include <stdint.h>

#include "robot_link_protocol.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        FRAME_PARSER_NONE = 0,
        FRAME_PARSER_FRAME,
        FRAME_PARSER_LENGTH_ERROR,
        FRAME_PARSER_CHECKSUM_ERROR
    } frame_parser_result_t;

    typedef enum
    {
        FRAME_PARSER_WAIT_HEAD0 = 0,
        FRAME_PARSER_WAIT_HEAD1,
        FRAME_PARSER_WAIT_LEN,
        FRAME_PARSER_WAIT_BODY
    } frame_parser_state_t;

    typedef struct
    {
        uint8_t cmd;
        uint8_t payload_len;
        uint8_t payload[ROBOT_LINK_PROTOCOL_MAX_PAYLOAD];
    } protocol_frame_t;

    typedef struct
    {
        frame_parser_state_t state;
        uint8_t              buffer[ROBOT_LINK_PROTOCOL_MAX_PAYLOAD + 3U];
        uint8_t              frame_len;
        uint8_t              frame_index;
    } frame_parser_t;

    /** Initialize a framed stream parser. */
    void FrameParser_Init(frame_parser_t *parser);

    /** Reset a parser to wait for the first header byte. */
    void FrameParser_Reset(frame_parser_t *parser);

    /** Return non-zero when the parser is waiting for a new frame. */
    uint8_t FrameParser_IsIdle(const frame_parser_t *parser);

    /** Push one byte and return the resulting parser event. */
    frame_parser_result_t FrameParser_Push(frame_parser_t *parser, uint8_t byte, protocol_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif
