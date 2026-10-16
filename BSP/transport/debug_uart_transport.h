#ifndef DEBUG_UART_TRANSPORT_H
#define DEBUG_UART_TRANSPORT_H

#include <stdint.h>

typedef struct
{
    uint32_t completed_bytes;
    uint32_t dropped_bytes;
    uint32_t dropped_writes;
    uint32_t busy_count;
    uint32_t error_count;
    uint16_t queued_bytes;
    uint16_t high_water_bytes;
    uint8_t  suspended;
} debug_uart_tx_status_t;

void DebugUartTransport_Init(void);
void DebugUartTransport_Write(const uint8_t *data, uint16_t size, uint32_t timeout_ms);
void DebugUartTransport_ProcessTx(void);
void DebugUartTransport_OnTxComplete(void);
/** Abort and discard console TX before another owner takes USART1. */
void     DebugUartTransport_Suspend(void);
void     DebugUartTransport_GetTxStatus(debug_uart_tx_status_t *status);
uint8_t  DebugUartTransport_ReadByte(uint8_t *byte);
void     DebugUartTransport_ClearRx(void);
void     DebugUartTransport_RestartRx(void);
uint8_t  DebugUartTransport_OnRxComplete(void);
void     DebugUartTransport_OnUartError(void);
uint32_t DebugUartTransport_GetOverflowCount(void);

#endif
