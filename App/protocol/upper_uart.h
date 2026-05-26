#ifndef UPPER_UART_H
#define UPPER_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint32_t tx_frames;
  uint32_t tx_busy_drops;
} upper_uart_state_t;

void UpperUart_Init(void);
void UpperUart_Update(void);
void Task_UpperUart(void *argument);
void UpperUart_GetState(upper_uart_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
