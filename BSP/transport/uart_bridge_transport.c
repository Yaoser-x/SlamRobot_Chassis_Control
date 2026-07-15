#include "uart_bridge_transport.h"

#include "platform_critical.h"
#include "platform_time.h"
#include "usart.h"

#define UART_BRIDGE_RING_SIZE     4096U
#define UART_BRIDGE_TX_CHUNK_SIZE 128U

typedef struct
{
    uint8_t           data[UART_BRIDGE_RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} uart_bridge_ring_t;

static uart_bridge_ring_t            pc_to_esp_ring;
static uart_bridge_ring_t            esp_to_pc_ring;
static uint8_t                       pc_to_esp_tx_chunk[UART_BRIDGE_TX_CHUNK_SIZE];
static uint8_t                       esp_to_pc_tx_chunk[UART_BRIDGE_TX_CHUNK_SIZE];
static uint8_t                       usart1_rx_byte;
static uint8_t                       usart2_rx_byte;
static volatile uint8_t              pc_to_esp_tx_busy;
static volatile uint8_t              esp_to_pc_tx_busy;
static uart_bridge_transport_state_t bridge_state;

static void UartBridgeRing_Clear(uart_bridge_ring_t *ring)
{
    platform_critical_state_t critical = PlatformCritical_Enter();

    ring->head = 0U;
    ring->tail = 0U;
    PlatformCritical_Exit(critical);
}

static uint8_t UartBridgeRing_Push(uart_bridge_ring_t *ring, uint8_t byte)
{
    uint16_t next_head = (uint16_t)((ring->head + 1U) % UART_BRIDGE_RING_SIZE);

    if (next_head == ring->tail)
    {
        return 0U;
    }
    ring->data[ring->head] = byte;
    ring->head             = next_head;
    return 1U;
}

static uint16_t UartBridgeRing_PopChunkLocked(uart_bridge_ring_t *ring, uint8_t *out, uint16_t max_len)
{
    uint16_t count = 0U;

    while (ring->tail != ring->head && count < max_len)
    {
        out[count++] = ring->data[ring->tail];
        ring->tail   = (uint16_t)((ring->tail + 1U) % UART_BRIDGE_RING_SIZE);
    }
    return count;
}

static void UartBridgeTransport_ResetBusy(void)
{
    platform_critical_state_t critical = PlatformCritical_Enter();

    pc_to_esp_tx_busy = 0U;
    esp_to_pc_tx_busy = 0U;
    PlatformCritical_Exit(critical);
}

static void UartBridgeTransport_StartPcToEspTx(void)
{
    platform_critical_state_t critical;
    uint16_t                  length = 0U;

    critical = PlatformCritical_Enter();
    if (pc_to_esp_tx_busy == 0U)
    {
        length =
            UartBridgeRing_PopChunkLocked(&pc_to_esp_ring, pc_to_esp_tx_chunk, (uint16_t)sizeof(pc_to_esp_tx_chunk));
        pc_to_esp_tx_busy = (length > 0U) ? 1U : 0U;
    }
    PlatformCritical_Exit(critical);

    if (length > 0U)
    {
        if (HAL_UART_Transmit_IT(&huart2, pc_to_esp_tx_chunk, length) == HAL_OK)
        {
            bridge_state.pc_to_esp_tx_bytes += length;
        }
        else
        {
            pc_to_esp_tx_busy = 0U;
            bridge_state.uart_error_count++;
        }
    }
}

static void UartBridgeTransport_StartEspToPcTx(void)
{
    platform_critical_state_t critical;
    uint16_t                  length = 0U;

    critical = PlatformCritical_Enter();
    if (esp_to_pc_tx_busy == 0U)
    {
        length =
            UartBridgeRing_PopChunkLocked(&esp_to_pc_ring, esp_to_pc_tx_chunk, (uint16_t)sizeof(esp_to_pc_tx_chunk));
        esp_to_pc_tx_busy = (length > 0U) ? 1U : 0U;
    }
    PlatformCritical_Exit(critical);

    if (length > 0U)
    {
        if (HAL_UART_Transmit_IT(&huart1, esp_to_pc_tx_chunk, length) == HAL_OK)
        {
            bridge_state.esp_to_pc_tx_bytes += length;
        }
        else
        {
            esp_to_pc_tx_busy = 0U;
            bridge_state.uart_error_count++;
        }
    }
}

void UartBridgeTransport_Init(void)
{
    bridge_state = (uart_bridge_transport_state_t){0};
    UartBridgeRing_Clear(&pc_to_esp_ring);
    UartBridgeRing_Clear(&esp_to_pc_ring);
    UartBridgeTransport_ResetBusy();
}

uint8_t UartBridgeTransport_Start(uint32_t initial_activity_ms)
{
    uint8_t rx_ok = 1U;

    (void)HAL_UART_Abort(&huart1);
    (void)HAL_UART_Abort(&huart2);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    UartBridgeRing_Clear(&pc_to_esp_ring);
    UartBridgeRing_Clear(&esp_to_pc_ring);
    UartBridgeTransport_ResetBusy();
    bridge_state                  = (uart_bridge_transport_state_t){0};
    bridge_state.active           = 1U;
    bridge_state.last_activity_ms = initial_activity_ms;

    if (HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1U) != HAL_OK)
    {
        bridge_state.rx_start_errors++;
        rx_ok = 0U;
    }
    if (HAL_UART_Receive_IT(&huart2, &usart2_rx_byte, 1U) != HAL_OK)
    {
        bridge_state.rx_start_errors++;
        rx_ok = 0U;
    }
    if (rx_ok == 0U)
    {
        (void)HAL_UART_Abort(&huart1);
        (void)HAL_UART_Abort(&huart2);
        UartBridgeTransport_ResetBusy();
        UartBridgeRing_Clear(&pc_to_esp_ring);
        UartBridgeRing_Clear(&esp_to_pc_ring);
        bridge_state.active = 0U;
    }
    return rx_ok;
}

void UartBridgeTransport_Stop(void)
{
    (void)HAL_UART_Abort(&huart1);
    (void)HAL_UART_Abort(&huart2);
    UartBridgeTransport_ResetBusy();
    UartBridgeRing_Clear(&pc_to_esp_ring);
    UartBridgeRing_Clear(&esp_to_pc_ring);
    bridge_state.active = 0U;
}

void UartBridgeTransport_Process(void)
{
    if (bridge_state.active == 0U)
    {
        return;
    }
    UartBridgeTransport_StartPcToEspTx();
    UartBridgeTransport_StartEspToPcTx();
}

uint8_t UartBridgeTransport_IsActive(void)
{
    platform_critical_state_t critical = PlatformCritical_Enter();
    uint8_t                   active   = bridge_state.active;

    PlatformCritical_Exit(critical);
    return active;
}

uint32_t UartBridgeTransport_GetIdleMs(void)
{
    platform_critical_state_t critical         = PlatformCritical_Enter();
    uint32_t                  last_activity_ms = bridge_state.last_activity_ms;
    int32_t                   elapsed_ms;

    PlatformCritical_Exit(critical);
    elapsed_ms = (int32_t)(PlatformTime_NowMs() - last_activity_ms);
    return (elapsed_ms > 0) ? (uint32_t)elapsed_ms : 0U;
}

void UartBridgeTransport_OnRx(uart_bridge_port_t port)
{
    if (bridge_state.active == 0U)
    {
        return;
    }
    bridge_state.last_activity_ms = PlatformTime_NowMs();
    if (port == UART_BRIDGE_PORT_PC)
    {
        if (UartBridgeRing_Push(&pc_to_esp_ring, usart1_rx_byte) != 0U)
        {
            bridge_state.pc_to_esp_rx_bytes++;
        }
        else
        {
            bridge_state.pc_to_esp_overflow++;
        }
        (void)HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1U);
    }
    else
    {
        if (UartBridgeRing_Push(&esp_to_pc_ring, usart2_rx_byte) != 0U)
        {
            bridge_state.esp_to_pc_rx_bytes++;
        }
        else
        {
            bridge_state.esp_to_pc_overflow++;
        }
        (void)HAL_UART_Receive_IT(&huart2, &usart2_rx_byte, 1U);
    }
}

void UartBridgeTransport_OnTxComplete(uart_bridge_port_t port)
{
    if (port == UART_BRIDGE_PORT_PC)
    {
        esp_to_pc_tx_busy = 0U;
        UartBridgeTransport_StartEspToPcTx();
    }
    else
    {
        pc_to_esp_tx_busy = 0U;
        UartBridgeTransport_StartPcToEspTx();
    }
}

void UartBridgeTransport_OnError(uart_bridge_port_t port)
{
    if (bridge_state.active == 0U)
    {
        return;
    }
    bridge_state.uart_error_count++;
    if (port == UART_BRIDGE_PORT_PC)
    {
        esp_to_pc_tx_busy = 0U;
        (void)HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1U);
    }
    else
    {
        pc_to_esp_tx_busy = 0U;
        (void)HAL_UART_Receive_IT(&huart2, &usart2_rx_byte, 1U);
    }
}

void UartBridgeTransport_GetState(uart_bridge_transport_state_t *state)
{
    if (state != 0)
    {
        platform_critical_state_t critical = PlatformCritical_Enter();

        *state = bridge_state;
        PlatformCritical_Exit(critical);
    }
}
