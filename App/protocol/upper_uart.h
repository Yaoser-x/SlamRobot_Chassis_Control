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
  uint32_t rx_checksum_errors;
  uint32_t rx_timeout_resets;
  uint32_t uart_errors;
  uint32_t rx_dma_half_count;
  uint32_t rx_dma_full_count;
  uint32_t rx_overwrite_count;
  uint32_t rx_resync_restarts;
  uint32_t last_valid_frame_ms;
} upper_uart_state_t;

void UpperUart_Init(void);
void UpperUart_Update(void);
void Task_UpperUart(void *argument);
void UpperUart_GetState(upper_uart_state_t *state);
uint32_t UpperUart_GetLastRxTimestamp(void);
void UpperUart_OnUartError(void);
void UpperUart_OnDmaHalf(void);
void UpperUart_OnDmaFull(void);
void UpperUart_OnTxComplete(void);

#ifdef __cplusplus
}
#endif

#endif
