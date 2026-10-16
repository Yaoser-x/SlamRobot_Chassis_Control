#include "debug_uart_transport.h"

#include "platform_critical.h"
#include "usart.h"

#define DEBUG_UART_RX_RING_SIZE  160U
#define DEBUG_UART_TX_RING_SIZE  4096U
#define DEBUG_UART_TX_CHUNK_SIZE 128U

static uint8_t                rx_byte;
static volatile uint8_t       rx_ring[DEBUG_UART_RX_RING_SIZE];
static volatile uint16_t      rx_head;
static volatile uint16_t      rx_tail;
static uint32_t               rx_overflow_count;
static uint8_t                tx_ring[DEBUG_UART_TX_RING_SIZE];
static uint16_t               tx_head;
static uint16_t               tx_tail;
static uint16_t               tx_inflight;
static debug_uart_tx_status_t tx_status;

void DebugUartTransport_Init(void)
{
    rx_head           = 0U;
    rx_tail           = 0U;
    rx_overflow_count = 0U;
    tx_head = tx_tail = tx_inflight = 0U;
    tx_status                       = (debug_uart_tx_status_t){0};
    HAL_NVIC_SetPriority(USART1_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    DebugUartTransport_RestartRx();
}

void DebugUartTransport_Write(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    platform_critical_state_t critical;

    (void)timeout_ms; /* Kept for source compatibility; enqueue never waits. */
    if (data == 0 || size == 0U)
    {
        return;
    }
    critical = PlatformCritical_Enter();
    if (tx_status.suspended != 0U || size > DEBUG_UART_TX_RING_SIZE - 1U - tx_status.queued_bytes)
    {
        tx_status.dropped_writes++;
        tx_status.dropped_bytes += size;
    }
    else
    {
        for (uint16_t index = 0U; index < size; ++index)
        {
            tx_ring[tx_head] = data[index];
            tx_head          = (uint16_t)((tx_head + 1U) % DEBUG_UART_TX_RING_SIZE);
        }
        tx_status.queued_bytes += size;
        if (tx_status.queued_bytes > tx_status.high_water_bytes)
        {
            tx_status.high_water_bytes = tx_status.queued_bytes;
        }
    }
    PlatformCritical_Exit(critical);
    DebugUartTransport_ProcessTx();
}

void DebugUartTransport_ProcessTx(void)
{
    platform_critical_state_t critical = PlatformCritical_Enter();
    if (tx_status.suspended == 0U && tx_inflight == 0U && tx_status.queued_bytes != 0U)
    {
        uint16_t          length = tx_status.queued_bytes;
        HAL_StatusTypeDef result;
        if (length > DEBUG_UART_TX_CHUNK_SIZE)
            length = DEBUG_UART_TX_CHUNK_SIZE;
        if (length > DEBUG_UART_TX_RING_SIZE - tx_tail)
            length = DEBUG_UART_TX_RING_SIZE - tx_tail;
        /* Tail stays reserved until TC: callers may reuse their buffers immediately. */
        tx_inflight = length;
        result      = HAL_UART_Transmit_IT(&huart1, &tx_ring[tx_tail], length);
        if (result != HAL_OK)
        {
            tx_inflight = 0U;
            if (result == HAL_BUSY)
                tx_status.busy_count++;
            else
                tx_status.error_count++;
        }
    }
    PlatformCritical_Exit(critical);
}

void DebugUartTransport_OnTxComplete(void)
{
    platform_critical_state_t critical = PlatformCritical_Enter();
    if (tx_status.suspended == 0U && tx_inflight != 0U)
    {
        tx_tail = (uint16_t)((tx_tail + tx_inflight) % DEBUG_UART_TX_RING_SIZE);
        tx_status.queued_bytes -= tx_inflight;
        tx_status.completed_bytes += tx_inflight;
        tx_inflight = 0U;
    }
    PlatformCritical_Exit(critical);
    DebugUartTransport_ProcessTx();
}

void DebugUartTransport_Suspend(void)
{
    platform_critical_state_t critical = PlatformCritical_Enter();
    tx_status.suspended                = 1U;
    (void)HAL_UART_AbortTransmit(&huart1);
    if (tx_status.queued_bytes != 0U)
    {
        tx_status.dropped_writes++;
        tx_status.dropped_bytes += tx_status.queued_bytes;
    }
    tx_head = tx_tail = tx_inflight = 0U;
    tx_status.queued_bytes          = 0U;
    PlatformCritical_Exit(critical);
}

void DebugUartTransport_GetTxStatus(debug_uart_tx_status_t *status)
{
    if (status != 0)
    {
        platform_critical_state_t critical = PlatformCritical_Enter();
        *status                            = tx_status;
        PlatformCritical_Exit(critical);
    }
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
    tx_status.suspended = 0U;
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
    tx_status.error_count++;
    (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}

uint32_t DebugUartTransport_GetOverflowCount(void)
{
    return rx_overflow_count;
}
