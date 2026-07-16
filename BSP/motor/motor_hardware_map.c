#include "motor_hardware_map.h"

/* CubeMX labels keep legacy M2/M3 names; logical M2/M3 nFAULT pins are crossed here. */
static const motor_hw_t motor_hw[MOTOR_ID_COUNT] = {
    {&htim1, TIM_CHANNEL_1, M1_IN2_GPIO_Port, M1_IN2_Pin, M1_FAULT_GPIO_Port, M1_FAULT_Pin},
    {&htim1, TIM_CHANNEL_2, M2_IN2_GPIO_Port, M2_IN2_Pin, M3_FAULT_GPIO_Port, M3_FAULT_Pin},
    {&htim1, TIM_CHANNEL_3, M3_IN2_GPIO_Port, M3_IN2_Pin, M2_FAULT_GPIO_Port, M2_FAULT_Pin},
    {&htim1, TIM_CHANNEL_4, M4_IN2_GPIO_Port, M4_IN2_Pin, M4_FAULT_GPIO_Port, M4_FAULT_Pin},
};

const motor_hw_t *MotorHardwareMap_Get(motor_id_t motor)
{
    return ((uint32_t)motor < MOTOR_ID_COUNT) ? &motor_hw[(uint32_t)motor] : 0;
}
