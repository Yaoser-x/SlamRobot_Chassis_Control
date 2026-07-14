#include "encoder_service.h"

#include "param_service.h"

void EncoderService_Init(void)
{
    EncoderDriver_Init();
}

void EncoderService_Update(uint32_t now_ms)
{
    param_model_t params;

    (void)ParamService_GetSnapshot(&params);
    EncoderDriver_Update(now_ms, &params);
}

void EncoderService_GetSnapshot(encoder_service_snapshot_t *snapshot)
{
    EncoderDriver_GetState(snapshot);
}
