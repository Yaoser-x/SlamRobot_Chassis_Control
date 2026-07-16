#ifndef ESP12F_SERVICE_H
#define ESP12F_SERVICE_H

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
    } esp12f_service_state_t;

    /** Initialize wireless communication with injected publication periods. */
    uint8_t Esp12fService_Init(const communication_config_t *config);
    void    Esp12fService_RestartRx(void);
    /** Poll wireless RX and publish frames from one App-provided coherent model. */
    void Esp12fService_Update(const communication_publish_model_t *publish_model);
    void Esp12fService_ResetModule(void);
    void Esp12fService_Isolate(void);
    void Esp12fService_SetDownloadMode(uint8_t enabled);
    void Esp12fService_OnRxCplt(void);
    void Esp12fService_OnUartError(void);
    void Esp12fService_GetState(esp12f_service_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
