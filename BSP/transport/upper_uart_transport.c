#include "upper_uart_transport.h"

#include "usart.h"

void UpperUartTransport_StartRx(uint8_t *buffer, uint16_t size)
{
    (void)HAL_UART_Receive_DMA(&huart3, buffer, size);
}

void UpperUartTransport_RestartRx(uint8_t *buffer, uint16_t size)
{
    (void)HAL_UART_DMAStop(&huart3);
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    __HAL_UART_CLEAR_NEFLAG(&huart3);
    __HAL_UART_CLEAR_FEFLAG(&huart3);
    __HAL_UART_CLEAR_PEFLAG(&huart3);
    (void)HAL_UART_Receive_DMA(&huart3, buffer, size);
}

uint16_t UpperUartTransport_GetRxWritePosition(uint16_t size)
{
    if (huart3.hdmarx == 0)
    {
        return size;
    }
    return (uint16_t)(size - __HAL_DMA_GET_COUNTER(huart3.hdmarx));
}

transport_status_t UpperUartTransport_TransmitAsync(uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef status = HAL_UART_Transmit_IT(&huart3, data, size);
    if (status == HAL_OK)
    {
        return TRANSPORT_STATUS_OK;
    }
    return (status == HAL_BUSY) ? TRANSPORT_STATUS_BUSY : TRANSPORT_STATUS_ERROR;
}
