#ifndef WHEEL_ENCODER_TYPES_H
#define WHEEL_ENCODER_TYPES_H

#include <stdint.h>

#define WHEEL_ENCODER_CHANNEL_COUNT 4U

/** @brief Raw timer facts captured from the wheel encoder peripherals. */
typedef struct
{
    uint32_t count[WHEEL_ENCODER_CHANNEL_COUNT];
    uint32_t period[WHEEL_ENCODER_CHANNEL_COUNT];
} wheel_encoder_sample_t;

#endif /* WHEEL_ENCODER_TYPES_H */
