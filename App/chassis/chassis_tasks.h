#ifndef CHASSIS_TASKS_H
#define CHASSIS_TASKS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ChassisTasks_InitHardware(void);
uint32_t ChassisTasks_GetMissedPeriodCount(uint32_t task);
void Task_Safety(void *argument);
void Task_MotorControl(void *argument);
void Task_RpiComm(void *argument);
void Task_Imu(void *argument);
void Task_Line(void *argument);
void Task_Esp12f(void *argument);
void Task_Usart1DebugConsole(void *argument);
void Task_Ps2(void *argument);
void Task_Led(void *argument);
void Task_Oled(void *argument);

#ifdef __cplusplus
}
#endif

#endif
