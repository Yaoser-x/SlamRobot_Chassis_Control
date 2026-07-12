#ifndef DIRECTION_APPLY_H
#define DIRECTION_APPLY_H

#include <stdint.h>

/** Apply a validated runtime direction to a signed motor or encoder value. */
static inline int32_t DirectionApply_Signed(int32_t value, int8_t direction)
{
    return value * (int32_t)direction;
}

#endif
