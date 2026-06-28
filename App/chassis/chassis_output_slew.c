#include "chassis_output_slew.h"

int16_t ChassisOutputSlew_Step(int16_t current, int16_t target, int16_t step)
{
  int32_t next;

  if (step <= 0)
  {
    return target;
  }

  if (current < target)
  {
    next = (int32_t)current + step;
    return (next > target) ? target : (int16_t)next;
  }
  if (current > target)
  {
    next = (int32_t)current - step;
    return (next < target) ? target : (int16_t)next;
  }
  return target;
}
