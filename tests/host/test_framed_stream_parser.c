#include "framed_stream_parser.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static frame_parser_result_t PushBytes(frame_parser_t       *parser,
                                       const uint8_t        *data,
                                       uint16_t              length,
                                       protocol_frame_t     *frame,
                                       frame_parser_result_t expected_terminal)
{
    frame_parser_result_t result = FRAME_PARSER_NONE;

    for (uint16_t index = 0U; index < length; ++index)
    {
        result = FrameParser_Push(parser, data[index], frame);
        if (index + 1U < length)
        {
            assert(result == FRAME_PARSER_NONE);
        }
    }
    assert(result == expected_terminal);
    return result;
}

int main(void)
{
    frame_parser_t   parser;
    protocol_frame_t frame = {0};
    uint8_t          bytes[ROBOT_LINK_PROTOCOL_MAX_FRAME];
    const uint8_t    payload[] = {0x11U, 0x22U, 0x33U};
    uint16_t         length;

    FrameParser_Init(&parser);
    assert(FrameParser_IsIdle(&parser) != 0U);
    assert(FrameParser_Push(&parser, 0x00U, &frame) == FRAME_PARSER_NONE);
    assert(FrameParser_Push(&parser, ROBOT_LINK_PROTOCOL_HEAD_0, &frame) == FRAME_PARSER_NONE);
    assert(FrameParser_Push(&parser, 0x00U, &frame) == FRAME_PARSER_NONE);
    assert(FrameParser_IsIdle(&parser) != 0U);

    assert(FrameParser_Push(&parser, ROBOT_LINK_PROTOCOL_HEAD_0, &frame) == FRAME_PARSER_NONE);
    assert(FrameParser_Push(&parser, ROBOT_LINK_PROTOCOL_HEAD_1, &frame) == FRAME_PARSER_NONE);
    assert(FrameParser_Push(&parser, 0U, &frame) == FRAME_PARSER_LENGTH_ERROR);
    assert(FrameParser_IsIdle(&parser) != 0U);

    length = UpperProtocol_BuildFrame(UPPER_CMD_LINE_CTRL, payload, sizeof(payload), bytes, sizeof(bytes));
    assert(length != 0U);
    PushBytes(&parser, bytes, length, &frame, FRAME_PARSER_FRAME);
    assert(frame.cmd == UPPER_CMD_LINE_CTRL);
    assert(frame.payload_len == sizeof(payload));
    assert(memcmp(frame.payload, payload, sizeof(payload)) == 0);

    bytes[length - 1U] ^= 0x01U;
    PushBytes(&parser, bytes, length, &frame, FRAME_PARSER_CHECKSUM_ERROR);
    bytes[length - 1U] ^= 0x01U;

    assert(FrameParser_Push(&parser, bytes[0], &frame) == FRAME_PARSER_NONE);
    assert(FrameParser_Push(&parser, bytes[1], &frame) == FRAME_PARSER_NONE);
    FrameParser_Reset(&parser);
    PushBytes(&parser, bytes, length, &frame, FRAME_PARSER_FRAME);

    for (uint8_t frame_index = 0U; frame_index < 2U; ++frame_index)
    {
        PushBytes(&parser, bytes, length, &frame, FRAME_PARSER_FRAME);
    }

    puts("PASS: framed stream parser");
    return 0;
}
