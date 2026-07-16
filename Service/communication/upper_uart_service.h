#ifndef UPPER_UART_SERVICE_H
#define UPPER_UART_SERVICE_H

#include <stdint.h>

#include "communication_publish_model_types.h"
#include "communication_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        uint32_t tx_frames;
        uint32_t tx_busy_drops;
        uint32_t rx_checksum_errors;
        uint32_t rx_timeout_resets;
        uint32_t uart_errors;
        uint32_t rx_dma_half_count;
        uint32_t rx_dma_full_count;
        uint32_t rx_overwrite_count;
        uint32_t rx_resync_restarts;
        uint32_t last_valid_frame_ms;
    } upper_uart_service_state_t;

    /** Initialize Host communication with injected publication periods. */
    uint8_t UpperUartService_Init(const communication_config_t *config);
    /** Poll Host RX and publish frames from one App-provided coherent model. */
    void     UpperUartService_Update(const communication_publish_model_t *publish_model);
    void     UpperUartService_GetState(upper_uart_service_state_t *state);
    uint32_t UpperUartService_GetLastRxTimestamp(void);
    void     UpperUartService_OnUartError(void);
    void     UpperUartService_OnDmaHalf(void);
    void     UpperUartService_OnDmaFull(void);
    void     UpperUartService_OnTxComplete(void);

#ifdef __cplusplus
}
#endif

#endif
