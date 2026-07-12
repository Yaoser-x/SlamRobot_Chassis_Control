#ifndef ESP12F_COMM_H
#define ESP12F_COMM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
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
  uint8_t boot_mode_download;
} esp12f_comm_state_t;

void Esp12fComm_Init(void);
void Esp12fComm_RestartRx(void);
void Esp12fComm_Update(void);
void Esp12fComm_ResetModule(void);
void Esp12fComm_Isolate(void);
void Esp12fComm_SetDownloadMode(uint8_t enabled);
void Esp12fComm_OnRxCplt(void);
void Esp12fComm_OnUartError(void);
void Esp12fComm_GetState(esp12f_comm_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
