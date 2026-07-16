#include "wheel_encoder_driver.h"

#include "tim.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
} wheel_encoder_hardware_t;

/* CubeMX labels keep the legacy M2/M3 names; logical IDs are mapped here. */
static const wheel_encoder_hardware_t wheel_encoder_hardware[WHEEL_ENCODER_CHANNEL_COUNT] = {
    {&htim2},
    {&htim4},
    {&htim3},
    {&htim5},
};

void WheelEncoderDriver_Init(void)
{
    for (uint32_t index = 0U; index < WHEEL_ENCODER_CHANNEL_COUNT; ++index)
    {
        (void)HAL_TIM_Encoder_Start(wheel_encoder_hardware[index].htim, TIM_CHANNEL_ALL);
        __HAL_TIM_SET_COUNTER(wheel_encoder_hardware[index].htim, 0U);
    }
}

void WheelEncoderDriver_Read(wheel_encoder_sample_t *sample)
{
    if (sample == 0)
    {
        return;
    }
    for (uint32_t index = 0U; index < WHEEL_ENCODER_CHANNEL_COUNT; ++index)
    {
        sample->count[index]  = __HAL_TIM_GET_COUNTER(wheel_encoder_hardware[index].htim);
        sample->period[index] = __HAL_TIM_GET_AUTORELOAD(wheel_encoder_hardware[index].htim);
    }
}

void WheelEncoderDriver_GetHardwareCounts(uint32_t counts[WHEEL_ENCODER_CHANNEL_COUNT])
{
    wheel_encoder_sample_t sample;

    if (counts == 0)
    {
        return;
    }
    WheelEncoderDriver_Read(&sample);
    for (uint32_t index = 0U; index < WHEEL_ENCODER_CHANNEL_COUNT; ++index)
    {
        counts[index] = sample.count[index];
    }
}
