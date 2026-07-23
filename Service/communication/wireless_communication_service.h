#ifndef WIRELESS_COMMUNICATION_H
#define WIRELESS_COMMUNICATION_H

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
        uint32_t                         rx_frames;
        uint32_t                         tx_frames;
        uint32_t                         tx_busy_drops;
        uint32_t                         rx_checksum_errors;
        uint32_t                         rx_length_errors;
        uint32_t                         rx_overflow_errors;
        uint32_t                         rx_timeout_resets;
        uint32_t                         uart_errors;
        uint32_t                         last_rx_timestamp_ms;
        uint8_t                          boot_mode_download;
        communication_session_snapshot_t session;
        communication_operation_result_t last_operation_result;
    } wireless_communication_state_t;

    /** Initialize wireless communication with injected publication periods. */
    uint8_t WirelessCommunication_Init(const communication_config_t            *config,
                                       const communication_firmware_identity_t *identity);
    void    WirelessCommunication_RestartRx(void);
    /** Poll wireless RX and publish frames from one App-provided coherent model. */
    void    WirelessCommunication_Update(const communication_publish_model_t *publish_model);
    void    WirelessCommunication_ResetModule(void);
    void    WirelessCommunication_Isolate(void);
    void    WirelessCommunication_SetDownloadMode(uint8_t enabled);
    void    WirelessCommunication_OnRxCplt(void);
    void    WirelessCommunication_OnUartError(void);
    void    WirelessCommunication_GetState(wireless_communication_state_t *state);
    uint8_t WirelessCommunication_TakeOperation(uint32_t now_ms, communication_operation_request_t *request);
    void    WirelessCommunication_CompleteOperation(const communication_operation_request_t *request,
                                                    communication_operation_stage_t          stage,
                                                    uint32_t                                 detail_mask,
                                                    uint32_t                                 now_ms);

#ifdef __cplusplus
}
#endif

#endif
