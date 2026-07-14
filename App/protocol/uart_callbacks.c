#include "esp12f_service.h"
#include "esp12f_flash_bridge.h"
#include "line_uart.h"
#include "upper_uart_service.h"
#include "usart1_debug_console.h"
#include "usart.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((Esp12fFlashBridge_IsActive() != 0U) && (huart == &huart1 || huart == &huart2))
    {
        Esp12fFlashBridge_OnRxCplt(huart);
    }
    else if (huart == &huart3)
    {
        UpperUartService_OnDmaFull();
    }
    else if (huart == &huart1)
    {
        Usart1DebugConsole_OnRxCplt();
    }
    else if (huart == &huart2)
    {
        Esp12fService_OnRxCplt();
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        UpperUartService_OnDmaHalf();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((Esp12fFlashBridge_IsActive() != 0U) && (huart == &huart1 || huart == &huart2))
    {
        Esp12fFlashBridge_OnTxCplt(huart);
    }
    else if (huart == &huart4)
    {
        LineUart_OnTxCplt();
    }
    else if (huart == &huart3)
    {
        UpperUartService_OnTxComplete();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((Esp12fFlashBridge_IsActive() != 0U) && (huart == &huart1 || huart == &huart2))
    {
        Esp12fFlashBridge_OnUartError(huart);
    }
    else if (huart == &huart1)
    {
        Usart1DebugConsole_OnUartError();
    }
    else if (huart == &huart2)
    {
        Esp12fService_OnUartError();
    }
    else if (huart == &huart3)
    {
        UpperUartService_OnUartError();
    }
    else if (huart == &huart4)
    {
        LineUart_OnUartError();
    }
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}
