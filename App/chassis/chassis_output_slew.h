#ifndef CHASSIS_OUTPUT_SLEW_H
#define CHASSIS_OUTPUT_SLEW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int16_t ChassisOutputSlew_Step(int16_t current, int16_t target, int16_t step);

#ifdef __cplusplus
}
#endif

#endif
