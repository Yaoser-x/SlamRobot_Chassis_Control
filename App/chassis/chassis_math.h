#ifndef CHASSIS_MATH_H
#define CHASSIS_MATH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ChassisMath_ResolveDifferentialTargets(float linear_x,
                                            float angular_z,
                                            float wheel_base_m,
                                            float *left_mps,
                                            float *right_mps);
uint8_t ChassisMath_ControlDt(uint32_t now_ms,
                              uint32_t *last_step_ms,
                              uint8_t *initialized,
                              float *dt_s);

#ifdef __cplusplus
}
#endif

#endif
