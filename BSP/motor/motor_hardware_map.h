#ifndef MOTOR_HW_MAP_H
#define MOTOR_HW_MAP_H

#include "main.h"
#include "motor_types.h"
#include "tim.h"

typedef struct
{
    TIM_HandleTypeDef *in1_htim;
    uint32_t           in1_channel;
    GPIO_TypeDef      *phase_port;
    uint16_t           phase_pin;
    GPIO_TypeDef      *fault_port;
    uint16_t           fault_pin;
} motor_hw_t;

/** Return the immutable board mapping for one logical motor. */
const motor_hw_t *MotorHardwareMap_Get(motor_id_t motor);

#endif
