#ifndef UART_BRIDGE_TRANSPORT_H
#define UART_BRIDGE_TRANSPORT_H

#include <stdint.h>

typedef enum
{
    UART_BRIDGE_PORT_PC = 0,
    UART_BRIDGE_PORT_ESP
} uart_bridge_port_t;

typedef struct
{
    uint8_t  active;
    uint32_t pc_to_esp_rx_bytes;
    uint32_t esp_to_pc_rx_bytes;
    uint32_t pc_to_esp_tx_bytes;
    uint32_t esp_to_pc_tx_bytes;
    uint32_t pc_to_esp_overflow;
    uint32_t esp_to_pc_overflow;
    uint32_t uart_error_count;
    uint32_t rx_start_errors;
    uint32_t last_activity_ms;
} uart_bridge_transport_state_t;

/** Initialize the UART1-to-UART2 bridge transport. */
void UartBridgeTransport_Init(void);

/** Start bridge reception after both UARTs have been isolated. */
uint8_t UartBridgeTransport_Start(uint32_t initial_activity_ms);

/** Abort both bridge UARTs and clear pending bridge data. */
void UartBridgeTransport_Stop(void);

/** Progress pending bridge transmissions. */
void UartBridgeTransport_Process(void);

/** Return non-zero while the bridge owns UART1 and UART2. */
uint8_t UartBridgeTransport_IsActive(void);

/** Return bridge idle duration using the platform HAL timebase. */
uint32_t UartBridgeTransport_GetIdleMs(void);

/** Handle one HAL receive-complete event for a bridge port. */
void UartBridgeTransport_OnRx(uart_bridge_port_t port);

/** Handle one HAL transmit-complete event for a bridge port. */
void UartBridgeTransport_OnTxComplete(uart_bridge_port_t port);

/** Handle one HAL UART error event for a bridge port. */
void UartBridgeTransport_OnError(uart_bridge_port_t port);

/** Copy current bridge transport counters. */
void UartBridgeTransport_GetState(uart_bridge_transport_state_t *state);

#endif
