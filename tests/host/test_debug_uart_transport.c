#include "debug_uart_transport.h"
#include "usart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef       huart1;
static const uint8_t    *pending;
static uint16_t          pending_size;
static HAL_StatusTypeDef transmit_result;
static uint8_t           wire[16384];
static size_t            wire_size;
static uint32_t          abort_count;

void HAL_NVIC_SetPriority(int irq, uint32_t preempt, uint32_t sub)
{
    assert(irq == USART1_IRQn && preempt == 7U && sub == 0U);
}
void HAL_NVIC_EnableIRQ(int irq)
{
    assert(irq == USART1_IRQn);
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    assert(uart == &huart1 && data != 0 && size == 1U);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t size)
{
    assert(uart == &huart1 && size > 0U && size <= 128U);
    if (transmit_result == HAL_OK)
    {
        assert(pending == 0);
        pending      = data;
        pending_size = size;
    }
    return transmit_result;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart)
{
    assert(uart == &huart1);
    pending      = 0;
    pending_size = 0;
    abort_count++;
    return HAL_OK;
}

static void Complete(void)
{
    assert(pending != 0 && wire_size + pending_size <= sizeof(wire));
    memcpy(wire + wire_size, pending, pending_size);
    wire_size += pending_size;
    pending      = 0;
    pending_size = 0;
    DebugUartTransport_OnTxComplete();
}
static void Drain(void)
{
    while (pending != 0)
        Complete();
}
static void Reset(void)
{
    pending         = 0;
    pending_size    = 0;
    wire_size       = 0;
    abort_count     = 0;
    transmit_result = HAL_OK;
    DebugUartTransport_Init();
}
static void TestLifetimeOrderWrapAndOverflow(void)
{
    uint8_t                source[3000];
    debug_uart_tx_status_t status;
    Reset();
    memset(source, 'A', sizeof(source));
    DebugUartTransport_Write(source, sizeof(source), 100U);
    memset(source, 'B', sizeof(source));
    DebugUartTransport_Write(source, sizeof(source), 100U); /* Whole write rejected. */
    Drain();
    assert(wire_size == 3000U);
    for (size_t i = 0; i < wire_size; ++i)
        assert(wire[i] == 'A');
    DebugUartTransport_Write(source, sizeof(source), 100U); /* Crosses ring boundary. */
    memset(source, 'C', sizeof(source));
    Drain();
    assert(wire_size == 6000U);
    for (size_t i = 3000; i < wire_size; ++i)
        assert(wire[i] == 'B');
    DebugUartTransport_GetTxStatus(&status);
    assert(status.completed_bytes == 6000U && status.queued_bytes == 0U);
    assert(status.dropped_writes == 1U && status.dropped_bytes == 3000U);
    assert(status.high_water_bytes == 3000U);
}
static void TestBusyErrorAndBridgeOwnership(void)
{
    debug_uart_tx_status_t status;
    Reset();
    transmit_result = HAL_BUSY;
    DebugUartTransport_Write((const uint8_t *)"first", 5U, 0U);
    transmit_result = HAL_ERROR;
    DebugUartTransport_ProcessTx();
    transmit_result = HAL_OK;
    DebugUartTransport_ProcessTx();
    DebugUartTransport_OnUartError(); /* RX error must not recycle in-flight memory. */
    Drain();
    assert(wire_size == 5U && memcmp(wire, "first", 5U) == 0);
    DebugUartTransport_GetTxStatus(&status);
    assert(status.busy_count == 1U && status.error_count == 2U);
    DebugUartTransport_Write((const uint8_t *)"old", 3U, 0U);
    DebugUartTransport_Suspend();
    DebugUartTransport_OnTxComplete(); /* Late callback cannot launch old debug text. */
    DebugUartTransport_Write((const uint8_t *)"bridge", 6U, 0U);
    assert(pending == 0 && abort_count == 1U);
    DebugUartTransport_RestartRx();
    DebugUartTransport_Write((const uint8_t *)"new", 3U, 0U);
    Drain();
    assert(wire_size == 8U && memcmp(wire + 5U, "new", 3U) == 0);
    DebugUartTransport_GetTxStatus(&status);
    assert(status.dropped_bytes == 9U && status.dropped_writes == 2U);
}
int main(void)
{
    TestLifetimeOrderWrapAndOverflow();
    TestBusyErrorAndBridgeOwnership();
    puts("PASS: bounded asynchronous debug UART and bridge handoff");
    return 0;
}
