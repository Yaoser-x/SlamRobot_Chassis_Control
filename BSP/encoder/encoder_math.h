#ifndef ENCODER_MATH_H
#define ENCODER_MATH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int32_t EncoderMath_DiffCount(uint32_t now, uint32_t last, uint32_t period);

#ifdef __cplusplus
}
#endif

#endif
