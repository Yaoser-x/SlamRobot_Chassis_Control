#include "chassis_math.h"

void ChassisMath_ResolveDifferentialTargets(float linear_x,
                                            float angular_z,
                                            float wheel_base_m,
                                            float *left_mps,
                                            float *right_mps)
{
  float half_track_omega = angular_z * wheel_base_m * 0.5f;

  if (left_mps != 0)
  {
    *left_mps = linear_x - half_track_omega;
  }
  if (right_mps != 0)
  {
    *right_mps = linear_x + half_track_omega;
  }
}
