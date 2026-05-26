#ifndef LINE_UART_H
#define LINE_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint32_t rx_bytes;
  uint32_t rx_frames;
  uint32_t overflow_count;
  uint16_t last_frame_len;
  uint8_t last_frame[32];
} line_uart_state_t;

void LineUart_Init(void);
void LineUart_Update(void);
void LineUart_GetState(line_uart_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
