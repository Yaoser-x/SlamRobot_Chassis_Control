#ifndef ESP_FRAME_PARSER_H
#define ESP_FRAME_PARSER_H

#include <stdint.h>
#include <string.h>

#define ESP_FRAME_HEAD0 0xAAU
#define ESP_FRAME_HEAD1 0x55U
#define ESP_FRAME_MAX_PAYLOAD 64U
#define ESP_FRAME_INTERBYTE_TIMEOUT_MS 100UL

typedef enum
{
  ESP_FRAME_WAIT_HEAD0 = 0,
  ESP_FRAME_WAIT_HEAD1,
  ESP_FRAME_WAIT_LEN,
  ESP_FRAME_WAIT_BODY
} esp_frame_parser_state_t;

typedef struct
{
  esp_frame_parser_state_t state;
  uint8_t frame[ESP_FRAME_MAX_PAYLOAD + 5U];
  uint8_t command_length;
  uint16_t body_index;
  uint32_t last_byte_ms;
  uint32_t timeout_count;
  uint32_t crc_error_count;
  uint32_t length_error_count;
  uint32_t uart_error_count;
} esp_frame_parser_t;

static inline uint8_t EspFrameParser_Crc8(const uint8_t *data, uint16_t length)
{
  uint8_t crc = 0U;
  while (length-- != 0U)
  {
    crc ^= *data++;
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      crc = (crc & 0x01U) ? (uint8_t)((crc >> 1) ^ 0x8CU) : (uint8_t)(crc >> 1);
    }
  }
  return crc;
}

static inline void EspFrameParser_Reset(esp_frame_parser_t *parser)
{
  parser->state = ESP_FRAME_WAIT_HEAD0;
  parser->command_length = 0U;
  parser->body_index = 0U;
  parser->last_byte_ms = 0U;
}

static inline void EspFrameParser_Init(esp_frame_parser_t *parser)
{
  memset(parser, 0, sizeof(*parser));
  EspFrameParser_Reset(parser);
}

static inline uint8_t EspFrameParser_Feed(esp_frame_parser_t *parser,
                                         uint8_t byte,
                                         uint32_t now_ms)
{
  parser->last_byte_ms = now_ms;
  switch (parser->state)
  {
    case ESP_FRAME_WAIT_HEAD0:
      if (byte == ESP_FRAME_HEAD0) { parser->state = ESP_FRAME_WAIT_HEAD1; }
      break;
    case ESP_FRAME_WAIT_HEAD1:
      if (byte == ESP_FRAME_HEAD1)
      {
        parser->state = ESP_FRAME_WAIT_LEN;
        parser->frame[0] = ESP_FRAME_HEAD0;
        parser->frame[1] = ESP_FRAME_HEAD1;
      }
      else
      {
        parser->state = (byte == ESP_FRAME_HEAD0) ? ESP_FRAME_WAIT_HEAD1 : ESP_FRAME_WAIT_HEAD0;
      }
      break;
    case ESP_FRAME_WAIT_LEN:
      if (byte >= 1U && byte <= (ESP_FRAME_MAX_PAYLOAD + 1U))
      {
        parser->command_length = byte;
        parser->body_index = 0U;
        parser->frame[2] = byte;
        parser->state = ESP_FRAME_WAIT_BODY;
      }
      else
      {
        parser->length_error_count++;
        EspFrameParser_Reset(parser);
      }
      break;
    case ESP_FRAME_WAIT_BODY:
      parser->frame[3U + parser->body_index++] = byte;
      if (parser->body_index >= (uint16_t)parser->command_length + 1U)
      {
        uint8_t expected = EspFrameParser_Crc8(&parser->frame[2],
                                               (uint16_t)parser->command_length + 1U);
        parser->state = ESP_FRAME_WAIT_HEAD0;
        if (expected == byte) { return 1U; }
        parser->crc_error_count++;
      }
      break;
    default:
      EspFrameParser_Reset(parser);
      break;
  }
  return 0U;
}

static inline void EspFrameParser_CheckTimeout(esp_frame_parser_t *parser, uint32_t now_ms)
{
  if (parser->state != ESP_FRAME_WAIT_HEAD0 && parser->last_byte_ms != 0U &&
      (uint32_t)(now_ms - parser->last_byte_ms) > ESP_FRAME_INTERBYTE_TIMEOUT_MS)
  {
    parser->timeout_count++;
    EspFrameParser_Reset(parser);
  }
}

static inline void EspFrameParser_OnUartError(esp_frame_parser_t *parser)
{
  parser->uart_error_count++;
  EspFrameParser_Reset(parser);
}

#endif
