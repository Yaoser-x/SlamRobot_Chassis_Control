#ifndef ENCODER_SERVICE_H
#define ENCODER_SERVICE_H

#include "encoder_driver.h"

typedef encoder_state_t encoder_service_snapshot_t;

void EncoderService_Init(void);
void EncoderService_Update(uint32_t now_ms);
void EncoderService_GetSnapshot(encoder_service_snapshot_t *snapshot);

#endif
