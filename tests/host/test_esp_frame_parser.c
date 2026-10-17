#include "esp_frame_parser.h"
#include "telemetry_encoder.h"
#include "robot_link_protocol.h"

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

static void check_stm32_frame(esp_frame_parser_t *parser, const uint8_t *frame, uint16_t length, const char *message)
{
    uint8_t complete = 0U;
    check(length >= 5U && length <= UINT8_MAX, "STM32 frame has a valid wire length");
    for (uint16_t index = 0U; index < length; ++index)
    {
        uint8_t received = EspFrameParser_Feed(parser, frame[index], 1000U + index);
        check(received == 0U || index == length - 1U, "fragmented frame completes only after its CRC");
        complete |= received;
    }
    check(complete != 0U, message);
    check(memcmp(parser->frame, frame, length) == 0, "ESP retains the complete STM32 wire frame");
}

static void test_real_stm32_telemetry(void)
{
    esp_frame_parser_t                parser;
    communication_publish_model_t     snapshot = {0};
    communication_firmware_identity_t identity = {0};
    uint8_t                           frame[ROBOT_LINK_PROTOCOL_MAX_FRAME];
    uint8_t                           payload[ROBOT_LINK_PROTOCOL_MAX_PAYLOAD] = {0};
    uint16_t                          length;

    EspFrameParser_Init(&parser);
    length = TelemetryEncoder_BuildHello(&identity, 0x0D663E2AUL, frame, sizeof(frame));
    check_stm32_frame(&parser, frame, length, "ESP accepts the actual STM32 HELLO header and CRC");

    snapshot.control.emergency_stop     = 1U;
    snapshot.safety.battery_voltage     = 12.1f;
    snapshot.chassis.motor_enabled_mask = 6U;
    snapshot.generation                 = 123U;
    length = TelemetryEncoder_BuildStatus(&snapshot, COMMUNICATION_LINK_ESP12F, frame, sizeof(frame));
    check(length == ROBOT_LINK_PROTOCOL_STATUS_PAYLOAD_LEN + 5U, "real STATUS contains the full 92-byte payload");
    check_stm32_frame(&parser, frame, length, "ESP accepts the full STM32 STATUS beyond 64 bytes");

    /* An RX fault must recover on the next valid telemetry frame without a reboot. */
    EspFrameParser_OnUartError(&parser);
    (void)EspFrameParser_Feed(&parser, 0x11U, 1500U);
    length = TelemetryEncoder_BuildDiagnostic(&snapshot, frame, sizeof(frame));
    check_stm32_frame(&parser, frame, length, "real DIAGNOSTIC recovers after UART error and noise");

    length = UpperProtocol_BuildFrame(UPPER_CMD_STATUS, payload, sizeof(payload), frame, sizeof(frame));
    check_stm32_frame(&parser, frame, length, "ESP accepts the canonical protocol maximum payload");
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

    test_real_stm32_telemetry();
    puts("PASS: ESP frame parser recovery");
    return 0;
}
