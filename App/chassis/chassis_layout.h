#ifndef CHASSIS_LAYOUT_H
#define CHASSIS_LAYOUT_H

#include <stdint.h>

#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 返回 CHASSIS_Mx_ENABLED 配置的启用状态。 */
uint8_t ChassisLayout_MotorEnabled(motor_id_t motor);

/* 返回电机左右侧归属，用于 left_mps/right_mps 分配。 */
motor_side_t ChassisLayout_MotorSide(motor_id_t motor);

/* 返回 PWM 方向修正值，归一化为 1 或 -1。 */
int8_t ChassisLayout_MotorDirection(motor_id_t motor);

/* 返回编码器方向修正值，归一化为 1 或 -1。 */
int8_t ChassisLayout_EncoderDirection(motor_id_t motor);

/* 统计指定侧已启用电机数量。 */
uint8_t ChassisLayout_SideMotorCount(motor_side_t side);

/* 左右两侧都至少有一路启用电机时返回 1。 */
uint8_t ChassisLayout_HasBothSides(void);

#ifdef __cplusplus
}
#endif

#endif
