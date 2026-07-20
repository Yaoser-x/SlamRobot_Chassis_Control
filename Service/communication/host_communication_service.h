#ifndef HOST_COMMUNICATION_H
#define HOST_COMMUNICATION_H

#include <stdint.h>

#include "communication_publish_model_types.h"
#include "communication_config.h"
#include "communication_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        uint32_t                         tx_frames;
        uint32_t                         tx_busy_drops;
        uint32_t                         rx_checksum_errors;
        uint32_t                         rx_timeout_resets;
        uint32_t                         uart_errors;
        uint32_t                         rx_dma_half_count;
        uint32_t                         rx_dma_full_count;
        uint32_t                         rx_overwrite_count;
        uint32_t                         rx_resync_restarts;
        uint32_t                         last_valid_frame_ms;
        communication_session_snapshot_t session;
    } host_communication_state_t;

    /** Initialize Host communication with injected publication periods. */
    uint8_t HostCommunication_Init(const communication_config_t            *config,
                                   const communication_firmware_identity_t *identity);
    /** Poll Host RX and publish frames from one App-provided coherent model. */
    void     HostCommunication_Update(const communication_publish_model_t *publish_model);
    void     HostCommunication_GetState(host_communication_state_t *state);
    uint32_t HostCommunication_GetLastRxTimestamp(void);
    void     HostCommunication_OnUartError(void);
    void     HostCommunication_OnDmaHalf(void);
    void     HostCommunication_OnDmaFull(void);
    void     HostCommunication_OnTxComplete(void);

#ifdef __cplusplus
}
#endif

#endif
