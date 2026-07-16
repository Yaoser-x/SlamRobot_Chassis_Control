#ifndef UPPER_UART_TRANSPORT_H
#define UPPER_UART_TRANSPORT_H

#include <stdint.h>

#include "transport_status.h"

void               HostUartTransport_StartRx(uint8_t *buffer, uint16_t size);
void               HostUartTransport_RestartRx(uint8_t *buffer, uint16_t size);
uint16_t           HostUartTransport_GetRxWritePosition(uint16_t size);
transport_status_t HostUartTransport_TransmitAsync(uint8_t *data, uint16_t size);

#endif
