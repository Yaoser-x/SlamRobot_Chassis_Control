#include "uart_bridge_transport.h"

#include "usart.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static USART_TypeDef usart1_instance;
static USART_TypeDef usart2_instance;
UART_HandleTypeDef   huart1 = {.Instance = &usart1_instance};
UART_HandleTypeDef   huart2 = {.Instance = &usart2_instance};
UART_HandleTypeDef   huart3;
UART_HandleTypeDef   huart4;

static uint8_t          *pc_rx_byte;
static uint8_t          *esp_rx_byte;
static HAL_StatusTypeDef pc_receive_status;
static HAL_StatusTypeDef esp_receive_status;
static uint8_t           pc_tx_data[128];
static uint8_t           esp_tx_data[128];
static uint16_t          pc_tx_length;
static uint16_t          esp_tx_length;
static uint32_t          abort_count;
static uint32_t          fake_tick;
static uint32_t          console_rx, console_tx, console_error, wireless_rx, wireless_error;
static uint32_t          host_full, host_half, host_tx, host_error, line_tx, line_error, irq_count;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *uart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart);
void USART1_IRQHandler(void);
/* Startup/restoration still hold the maintenance lease even with no transport. */
uint8_t Esp12fFlashBridge_IsActive(void) { return 1U; }
void Usart1DebugConsole_OnRxCplt(void) { console_rx++; }
void Usart1DebugConsole_OnUartError(void) { console_error++; }
void DebugUartTransport_OnTxComplete(void) { console_tx++; }
void WirelessCommunication_OnRxCplt(void) { wireless_rx++; }
void WirelessCommunication_OnUartError(void) { wireless_error++; }
void HostCommunication_OnDmaFull(void) { host_full++; }
void HostCommunication_OnDmaHalf(void) { host_half++; }
void HostCommunication_OnTxComplete(void) { host_tx++; }
void HostCommunication_OnUartError(void) { host_error++; }
void LineSensorDriver_OnTxCplt(void) { line_tx++; }
void LineSensorDriver_OnUartError(void) { line_error++; }
void HAL_UART_IRQHandler(UART_HandleTypeDef *uart) { assert(uart == &huart1); irq_count++; }

uint32_t HAL_GetTick(void)
{
    return fake_tick;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    assert(size == 1U);
    if (uart == &huart1)
    {
        pc_rx_byte = data;
        return pc_receive_status;
    }
    esp_rx_byte = data;
    return esp_receive_status;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t size)
{
    if (uart == &huart1)
    {
        memcpy(pc_tx_data, data, size);
        pc_tx_length = size;
    }
    else
    {
        memcpy(esp_tx_data, data, size);
        esp_tx_length = size;
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *uart)
{
    (void)uart;
    abort_count++;
    return HAL_OK;
}

static void ResetFakes(void)
{
    pc_rx_byte         = 0;
    esp_rx_byte        = 0;
    pc_receive_status  = HAL_OK;
    esp_receive_status = HAL_OK;
    pc_tx_length       = 0U;
    esp_tx_length      = 0U;
    abort_count        = 0U;
    fake_tick          = 100U;
    usart1_instance    = (USART_TypeDef){0};
    usart2_instance    = (USART_TypeDef){0};
    UartBridgeTransport_Init();
}

static void TestBidirectionalBridgeAndIdle(void)
{
    uart_bridge_transport_state_t state;

    ResetFakes();
    assert(UartBridgeTransport_Start(100U) != 0U);
    assert(pc_rx_byte != 0 && esp_rx_byte != 0);

    *pc_rx_byte = 0x11U;
    UartBridgeTransport_OnRx(UART_BRIDGE_PORT_PC);
    UartBridgeTransport_Process();
    assert(esp_tx_length == 1U && esp_tx_data[0] == 0x11U);
    UartBridgeTransport_OnTxComplete(UART_BRIDGE_PORT_ESP);

    *esp_rx_byte = 0x22U;
    UartBridgeTransport_OnRx(UART_BRIDGE_PORT_ESP);
    UartBridgeTransport_Process();
    assert(pc_tx_length == 1U && pc_tx_data[0] == 0x22U);

    fake_tick = 250U;
    assert(UartBridgeTransport_GetIdleMs() == 150U);
    UartBridgeTransport_GetState(&state);
    assert(state.pc_to_esp_rx_bytes == 1U && state.pc_to_esp_tx_bytes == 1U);
    assert(state.esp_to_pc_rx_bytes == 1U && state.esp_to_pc_tx_bytes == 1U);

    UartBridgeTransport_Stop();
    assert(UartBridgeTransport_IsActive() == 0U);
    assert(abort_count >= 4U);
}

static void TestOverflowErrorAndStartFailure(void)
{
    uart_bridge_transport_state_t state;

    ResetFakes();
    assert(UartBridgeTransport_Start(100U) != 0U);
    for (uint16_t index = 0U; index < 4096U; ++index)
    {
        *pc_rx_byte = (uint8_t)index;
        UartBridgeTransport_OnRx(UART_BRIDGE_PORT_PC);
    }
    UartBridgeTransport_OnError(UART_BRIDGE_PORT_ESP);
    UartBridgeTransport_GetState(&state);
    assert(state.pc_to_esp_overflow == 1U);
    assert(state.uart_error_count == 1U);

    UartBridgeTransport_Stop();
    pc_receive_status = HAL_ERROR;
    assert(UartBridgeTransport_Start(500U) == 0U);
    UartBridgeTransport_GetState(&state);
    assert(state.active == 0U);
    assert(state.rx_start_errors == 1U);
}

static void TestCallbacksFollowTheActualTransportOwner(void)
{
    uart_bridge_transport_state_t state;
    ResetFakes();
    HAL_UART_RxCpltCallback(&huart1);
    HAL_UART_RxCpltCallback(&huart2);
    HAL_UART_TxCpltCallback(&huart1);
    HAL_UART_ErrorCallback(&huart1);
    HAL_UART_ErrorCallback(&huart2);
    assert(console_rx == 1U && wireless_rx == 1U && console_tx == 1U);
    assert(console_error == 1U && wireless_error == 1U);
    assert(UartBridgeTransport_Start(100U) != 0U);
    *pc_rx_byte = 0x44U;
    *esp_rx_byte = 0x55U;
    HAL_UART_RxCpltCallback(&huart1);
    HAL_UART_RxCpltCallback(&huart2);
    UartBridgeTransport_Process();
    HAL_UART_TxCpltCallback(&huart1);
    HAL_UART_TxCpltCallback(&huart2);
    HAL_UART_ErrorCallback(&huart1);
    HAL_UART_ErrorCallback(&huart2);
    assert(console_rx == 1U && wireless_rx == 1U && console_tx == 1U);
    assert(console_error == 1U && wireless_error == 1U);
    assert(pc_tx_length == 1U && pc_tx_data[0] == 0x55U);
    assert(esp_tx_length == 1U && esp_tx_data[0] == 0x44U);
    UartBridgeTransport_GetState(&state);
    assert(state.uart_error_count == 2U);
    UartBridgeTransport_Stop();
    HAL_UART_RxCpltCallback(&huart1);
    HAL_UART_RxCpltCallback(&huart2);
    HAL_UART_TxCpltCallback(&huart1);
    assert(console_rx == 2U && wireless_rx == 2U && console_tx == 2U);
    HAL_UART_RxCpltCallback(&huart3);
    HAL_UART_RxHalfCpltCallback(&huart3);
    HAL_UART_TxCpltCallback(&huart3);
    HAL_UART_ErrorCallback(&huart3);
    HAL_UART_TxCpltCallback(&huart4);
    HAL_UART_ErrorCallback(&huart4);
    USART1_IRQHandler();
    assert(host_full == 1U && host_half == 1U && host_tx == 1U && host_error == 1U);
    assert(line_tx == 1U && line_error == 1U && irq_count == 1U);
}

int main(void)
{
    TestBidirectionalBridgeAndIdle();
    TestOverflowErrorAndStartFailure();
    TestCallbacksFollowTheActualTransportOwner();
    puts("PASS: UART bridge transport");
    return 0;
}
