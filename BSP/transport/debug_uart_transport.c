#include "debug_uart_transport.h"

#include "platform_critical.h"
#include "usart.h"

#define DEBUG_UART_RX_RING_SIZE 160U

static uint8_t           rx_byte;
static volatile uint8_t  rx_ring[DEBUG_UART_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static uint32_t          rx_overflow_count;

void DebugUartTransport_Init(void)
{
    rx_head           = 0U;
    rx_tail           = 0U;
    rx_overflow_count = 0U;
    HAL_NVIC_SetPriority(USART1_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    DebugUartTransport_RestartRx();
}

void DebugUartTransport_Write(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)data, size, timeout_ms);
}

uint8_t DebugUartTransport_ReadByte(uint8_t *byte)
{
    if (byte == 0 || rx_tail == rx_head)
    {
        return 0U;
    }
    *byte   = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % DEBUG_UART_RX_RING_SIZE);
    return 1U;
}

void DebugUartTransport_ClearRx(void)
{
    platform_critical_state_t state = PlatformCritical_Enter();

    rx_head = 0U;
    rx_tail = 0U;
    PlatformCritical_Exit(state);
}

void DebugUartTransport_RestartRx(void)
{
    DebugUartTransport_ClearRx();
    (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}

uint8_t DebugUartTransport_OnRxComplete(void)
{
    uint16_t next_head = (uint16_t)((rx_head + 1U) % DEBUG_UART_RX_RING_SIZE);
    uint8_t  overflow  = 0U;

    if (next_head != rx_tail)
    {
        rx_ring[rx_head] = rx_byte;
        rx_head          = next_head;
    }
    else
    {
        rx_overflow_count++;
        overflow = 1U;
    }
    (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
    return overflow;
}

void DebugUartTransport_OnUartError(void)
{
    (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}

uint32_t DebugUartTransport_GetOverflowCount(void)
{
    return rx_overflow_count;
}
