#ifndef APP_TASKS_H
#define APP_TASKS_H

void Task_Safety(void *argument);
void Task_MotorControl(void *argument);
void Task_RpiComm(void *argument);
void Task_Imu(void *argument);
void Task_Line(void *argument);
void Task_Esp12f(void *argument);
void Task_Ps2(void *argument);
void Task_Led(void *argument);
void Task_Oled(void *argument);
void Task_Debug(void *argument);

#endif
