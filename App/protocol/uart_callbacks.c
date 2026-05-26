#include "esp12f_comm.h"
#include "esp12f_flash_bridge.h"
#include "usart1_debug_console.h"
#include "usart.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((Esp12fFlashBridge_IsActive() != 0U) && (huart == &huart1 || huart == &huart2))
  {
    Esp12fFlashBridge_OnRxCplt(huart);
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
}

void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}
