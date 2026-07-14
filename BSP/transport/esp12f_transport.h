#ifndef ESP12F_TRANSPORT_H
#define ESP12F_TRANSPORT_H

#include <stdint.h>

#include "transport_status.h"

uint8_t            Esp12fTransport_IsTxReady(void);
transport_status_t Esp12fTransport_TransmitAsync(uint8_t *data, uint16_t size);
void               Esp12fTransport_StartRx(uint8_t *byte);
void               Esp12fTransport_ResetModule(void);
void               Esp12fTransport_Isolate(void);
void               Esp12fTransport_SetDownloadMode(uint8_t enabled);

#endif
