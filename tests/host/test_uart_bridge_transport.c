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

int main(void)
{
    TestBidirectionalBridgeAndIdle();
    TestOverflowErrorAndStartFailure();
    puts("PASS: UART bridge transport");
    return 0;
}
