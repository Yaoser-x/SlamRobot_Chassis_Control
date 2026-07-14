#ifndef DEBUG_UART_TRANSPORT_H
#define DEBUG_UART_TRANSPORT_H

#include <stdint.h>

void DebugUartTransport_Write(const uint8_t *data, uint16_t size, uint32_t timeout_ms);

#endif
