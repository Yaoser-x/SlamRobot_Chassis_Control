#ifndef DEBUG_UART_TRANSPORT_H
#define DEBUG_UART_TRANSPORT_H

#include <stdint.h>

void     DebugUartTransport_Init(void);
void     DebugUartTransport_Write(const uint8_t *data, uint16_t size, uint32_t timeout_ms);
uint8_t  DebugUartTransport_ReadByte(uint8_t *byte);
void     DebugUartTransport_ClearRx(void);
void     DebugUartTransport_RestartRx(void);
uint8_t  DebugUartTransport_OnRxComplete(void);
void     DebugUartTransport_OnUartError(void);
uint32_t DebugUartTransport_GetOverflowCount(void);

#endif
