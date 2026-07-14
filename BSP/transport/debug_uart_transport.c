#include "debug_uart_transport.h"

#include "usart.h"

void DebugUartTransport_Write(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)data, size, timeout_ms);
}
