#ifndef CHASSIS_MATH_H
#define CHASSIS_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

void ChassisMath_ResolveDifferentialTargets(float linear_x,
                                            float angular_z,
                                            float wheel_base_m,
                                            float *left_mps,
                                            float *right_mps);

#ifdef __cplusplus
}
#endif

#endif
