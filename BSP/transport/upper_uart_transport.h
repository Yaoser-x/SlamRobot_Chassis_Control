#ifndef UPPER_UART_TRANSPORT_H
#define UPPER_UART_TRANSPORT_H

#include <stdint.h>

#include "transport_status.h"

void               UpperUartTransport_StartRx(uint8_t *buffer, uint16_t size);
void               UpperUartTransport_RestartRx(uint8_t *buffer, uint16_t size);
uint16_t           UpperUartTransport_GetRxWritePosition(uint16_t size);
transport_status_t UpperUartTransport_TransmitAsync(uint8_t *data, uint16_t size);

#endif
