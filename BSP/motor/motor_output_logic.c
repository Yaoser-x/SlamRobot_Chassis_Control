#include "motor_output_logic.h"

motor_output_phase_enable_t MotorOutputLogic_ResolvePhaseEnable(int16_t signed_permille)
{
  motor_output_phase_enable_t output = {0};

  if (signed_permille > 0)
  {
    output.en_permille = signed_permille;
    output.phase_high = 1U;
  }
  else if (signed_permille < 0)
  {
    output.en_permille = (int16_t)-signed_permille;
    output.phase_high = 0U;
  }

  return output;
}

motor_output_phase_enable_t MotorOutputLogic_ResolveRawInput(int16_t forward_permille,
                                                             int16_t reverse_permille)
{
  return MotorOutputLogic_ResolvePhaseEnable((int16_t)(forward_permille - reverse_permille));
}
