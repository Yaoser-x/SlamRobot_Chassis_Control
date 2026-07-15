#include "framed_stream_parser.h"

#include <string.h>

void FrameParser_Init(frame_parser_t *parser)
{
    FrameParser_Reset(parser);
}

void FrameParser_Reset(frame_parser_t *parser)
{
    if (parser == 0)
    {
        return;
    }
    parser->state       = FRAME_PARSER_WAIT_HEAD0;
    parser->frame_len   = 0U;
    parser->frame_index = 0U;
}

uint8_t FrameParser_IsIdle(const frame_parser_t *parser)
{
    return (parser != 0 && parser->state == FRAME_PARSER_WAIT_HEAD0) ? 1U : 0U;
}

frame_parser_result_t FrameParser_Push(frame_parser_t *parser, uint8_t byte, protocol_frame_t *frame)
{
    if (parser == 0)
    {
        return FRAME_PARSER_NONE;
    }

    switch (parser->state)
    {
        case FRAME_PARSER_WAIT_HEAD0:
            if (byte == UPPER_PROTOCOL_HEAD_0)
            {
                parser->state = FRAME_PARSER_WAIT_HEAD1;
            }
            break;

        case FRAME_PARSER_WAIT_HEAD1:
            parser->state = (byte == UPPER_PROTOCOL_HEAD_1) ? FRAME_PARSER_WAIT_LEN : FRAME_PARSER_WAIT_HEAD0;
            break;

        case FRAME_PARSER_WAIT_LEN:
            if (byte == 0U || byte > UPPER_PROTOCOL_CMD_LEN(UPPER_PROTOCOL_MAX_PAYLOAD))
            {
                FrameParser_Reset(parser);
                return FRAME_PARSER_LENGTH_ERROR;
            }
            parser->buffer[0]   = byte;
            parser->frame_len   = byte;
            parser->frame_index = 0U;
            parser->state       = FRAME_PARSER_WAIT_BODY;
            break;

        case FRAME_PARSER_WAIT_BODY:
            parser->frame_index++;
            parser->buffer[parser->frame_index] = byte;
            if (parser->frame_index >= (uint8_t)(parser->frame_len + 1U))
            {
                uint8_t checksum = parser->buffer[parser->frame_index];
                uint8_t expected = UpperProtocol_Checksum8(parser->buffer, (uint16_t)parser->frame_len + 1U);

                if (checksum != expected)
                {
                    FrameParser_Reset(parser);
                    return FRAME_PARSER_CHECKSUM_ERROR;
                }
                if (frame != 0)
                {
                    frame->cmd         = parser->buffer[1];
                    frame->payload_len = (uint8_t)(parser->frame_len - 1U);
                    if (frame->payload_len > 0U)
                    {
                        (void)memcpy(frame->payload, &parser->buffer[2], frame->payload_len);
                    }
                }
                FrameParser_Reset(parser);
                return FRAME_PARSER_FRAME;
            }
            break;

        default:
            FrameParser_Reset(parser);
            break;
    }
    return FRAME_PARSER_NONE;
}
