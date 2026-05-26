#include "encoder_math.h"

int32_t EncoderMath_DiffCount(uint32_t now, uint32_t last, uint32_t period)
{
  if (period <= 0xFFFFU)
  {
    return (int32_t)(int16_t)((uint16_t)now - (uint16_t)last);
  }
  return (int32_t)(now - last);
}
