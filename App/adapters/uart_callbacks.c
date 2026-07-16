#include "wireless_communication_service.h"
#include "esp12f_flash_bridge.h"
#include "line_sensor_driver.h"
#include "host_communication_service.h"
#include "uart_bridge_transport.h"
#include "usart1_debug_console.h"
#include "usart.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((Esp12fFlashBridge_IsActive() != 0U) && (huart == &huart1 || huart == &huart2))
    {
        UartBridgeTransport_OnRx((huart == &huart1) ? UART_BRIDGE_PORT_PC : UART_BRIDGE_PORT_ESP);
    }
    else if (huart == &huart3)
    {
        HostCommunication_OnDmaFull();
    }
    else if (huart == &huart1)
    {
        Usart1DebugConsole_OnRxCplt();
    }
    else if (huart == &huart2)
    {
        WirelessCommunication_OnRxCplt();
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        HostCommunication_OnDmaHalf();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((Esp12fFlashBridge_IsActive() != 0U) && (huart == &huart1 || huart == &huart2))
    {
        UartBridgeTransport_OnTxComplete((huart == &huart1) ? UART_BRIDGE_PORT_PC : UART_BRIDGE_PORT_ESP);
    }
    else if (huart == &huart4)
    {
        LineSensorDriver_OnTxCplt();
    }
    else if (huart == &huart3)
    {
        HostCommunication_OnTxComplete();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((Esp12fFlashBridge_IsActive() != 0U) && (huart == &huart1 || huart == &huart2))
    {
        UartBridgeTransport_OnError((huart == &huart1) ? UART_BRIDGE_PORT_PC : UART_BRIDGE_PORT_ESP);
    }
    else if (huart == &huart1)
    {
        Usart1DebugConsole_OnUartError();
    }
    else if (huart == &huart2)
    {
        WirelessCommunication_OnUartError();
    }
    else if (huart == &huart3)
    {
        HostCommunication_OnUartError();
    }
    else if (huart == &huart4)
    {
        LineSensorDriver_OnUartError();
    }
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}
