#ifndef WIRELESS_COMMUNICATION_H
#define WIRELESS_COMMUNICATION_H

#include <stdint.h>

#include "communication_publish_model_types.h"
#include "communication_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        uint32_t rx_frames;
        uint32_t tx_frames;
        uint32_t tx_busy_drops;
        uint32_t rx_checksum_errors;
        uint32_t rx_length_errors;
        uint32_t rx_overflow_errors;
        uint32_t rx_timeout_resets;
        uint32_t uart_errors;
        uint32_t last_rx_timestamp_ms;
        uint8_t  boot_mode_download;
    } wireless_communication_state_t;

    /** Initialize wireless communication with injected publication periods. */
    uint8_t WirelessCommunication_Init(const communication_config_t *config);
    void    WirelessCommunication_RestartRx(void);
    /** Poll wireless RX and publish frames from one App-provided coherent model. */
    void WirelessCommunication_Update(const communication_publish_model_t *publish_model);
    void WirelessCommunication_ResetModule(void);
    void WirelessCommunication_Isolate(void);
    void WirelessCommunication_SetDownloadMode(uint8_t enabled);
    void WirelessCommunication_OnRxCplt(void);
    void WirelessCommunication_OnUartError(void);
    void WirelessCommunication_GetState(wireless_communication_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
