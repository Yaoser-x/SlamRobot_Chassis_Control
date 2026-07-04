#include "esp12f_comm.h"
#include "esp12f_flash_bridge.h"
#include "line_uart.h"
#include "upper_uart.h"
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
    UpperUart_OnDmaFull();
  }
  else if (huart == &huart1)
  {
    Usart1DebugConsole_OnRxCplt();
  }
  else if (huart == &huart2)
  {
    Esp12fComm_OnRxCplt();
  }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    UpperUart_OnDmaHalf();
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
    Esp12fComm_OnUartError();
  }
  else if (huart == &huart3)
  {
    UpperUart_OnUartError();
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
