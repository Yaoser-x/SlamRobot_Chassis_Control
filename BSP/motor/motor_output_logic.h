#ifndef MOTOR_OUTPUT_LOGIC_H
#define MOTOR_OUTPUT_LOGIC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        int16_t en_permille;
        uint8_t phase_high;
    } motor_output_phase_enable_t;

    motor_output_phase_enable_t MotorOutputLogic_ResolvePhaseEnable(int16_t signed_permille);

#ifdef __cplusplus
}
#endif

#endif
