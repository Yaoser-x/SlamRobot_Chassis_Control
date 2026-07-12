#include "esp_frame_parser.h"

#include <stdio.h>
#include <stdlib.h>

static void check(int ok, const char *message)
{
    if (!ok)
    {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

static uint8_t build_frame(uint8_t command, uint8_t value, uint8_t frame[7])
{
    frame[0] = ESP_FRAME_HEAD0;
    frame[1] = ESP_FRAME_HEAD1;
    frame[2] = 2U;
    frame[3] = command;
    frame[4] = value;
    frame[5] = EspFrameParser_Crc8(&frame[2], 3U);
    return 6U;
}

static uint8_t feed_frame(esp_frame_parser_t *parser, const uint8_t *frame, uint8_t length, uint32_t start_ms)
{
    uint8_t complete = 0U;
    for (uint8_t i = 0U; i < length; ++i)
    {
        complete |= EspFrameParser_Feed(parser, frame[i], start_ms + i);
    }
    return complete;
}

int main(void)
{
    esp_frame_parser_t parser;
    uint8_t            frame[7];
    uint8_t            length = build_frame(0x81U, 0x42U, frame);

    EspFrameParser_Init(&parser);
    check(feed_frame(&parser, frame, length, 10U) == 1U, "valid frame accepted");
    check(parser.frame[3] == 0x81U && parser.frame[4] == 0x42U, "frame body retained");

    EspFrameParser_Init(&parser);
    (void)feed_frame(&parser, frame, 4U, 100U);
    EspFrameParser_CheckTimeout(&parser, 204U);
    check(parser.timeout_count == 1U, "truncated frame times out");
    check(feed_frame(&parser, frame, length, 300U) == 1U, "frame recovers after timeout");

    EspFrameParser_Init(&parser);
    (void)EspFrameParser_Feed(&parser, 0x11U, 1U);
    (void)EspFrameParser_Feed(&parser, ESP_FRAME_HEAD0, 2U);
    (void)EspFrameParser_Feed(&parser, ESP_FRAME_HEAD0, 3U);
    check(feed_frame(&parser, &frame[1], (uint8_t)(length - 1U), 4U) == 1U,
          "noise and repeated head recover without discarding next frame");

    EspFrameParser_Init(&parser);
    (void)EspFrameParser_Feed(&parser, ESP_FRAME_HEAD0, 1U);
    (void)EspFrameParser_Feed(&parser, ESP_FRAME_HEAD1, 2U);
    (void)EspFrameParser_Feed(&parser, 0U, 3U);
    check(parser.length_error_count == 1U, "illegal length counted");
    check(feed_frame(&parser, frame, length, 10U) == 1U, "frame recovers after length error");

    EspFrameParser_Init(&parser);
    frame[length - 1U] ^= 0x5AU;
    check(feed_frame(&parser, frame, length, 10U) == 0U && parser.crc_error_count == 1U,
          "CRC error rejected and counted");
    frame[length - 1U] ^= 0x5AU;
    check(feed_frame(&parser, frame, length, 30U) == 1U, "frame recovers after CRC error");

    EspFrameParser_Init(&parser);
    (void)feed_frame(&parser, frame, 4U, 50U);
    EspFrameParser_OnUartError(&parser);
    check(parser.uart_error_count == 1U, "UART error counted");
    check(feed_frame(&parser, frame, length, 70U) == 1U, "frame recovers after UART error");

    puts("PASS: ESP frame parser recovery");
    return 0;
}
