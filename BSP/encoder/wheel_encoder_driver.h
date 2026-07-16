#ifndef WHEEL_ENCODER_DRIVER_H
#define WHEEL_ENCODER_DRIVER_H

#include "wheel_encoder_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief Start and zero all configured encoder timer peripherals. */
    void WheelEncoderDriver_Init(void);
    /** @brief Capture raw counter and auto-reload values without estimating speed. */
    void WheelEncoderDriver_Read(wheel_encoder_sample_t *sample);
    /** @brief Read raw hardware timer counts for board diagnostics. */
    void WheelEncoderDriver_GetHardwareCounts(uint32_t counts[WHEEL_ENCODER_CHANNEL_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_ENCODER_DRIVER_H */
