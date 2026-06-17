#include "chassis_tasks.h"

#include "iwdg.h"
#include "adc_monitor.h"
#include "chassis_config.h"
#include "chassis_control.h"
#include "chassis_task_timing.h"
#include "cmsis_os2.h"
#include "encoder_driver.h"
#include "esp12f_comm.h"
#include "esp12f_flash_bridge.h"
#include "imu_bmi270.h"
#include "led_status.h"
#include "line_control.h"
#include "line_uart.h"
#include "ps2_control.h"
#include "system_monitor.h"
#include "upper_uart.h"
#include "usart1_debug_console.h"
#include "ssd1306.h"
#include "oled_ui.h"

void ChassisTasks_InitHardware(void)
{
  ChassisTaskTiming_Reset();
  EncoderDriver_Init();
  AdcMonitor_Init();
  ImuBmi270_Init();
  LedStatus_Init();
  SystemMonitor_Init();
  ChassisControl_Init();
  UpperUart_Init();
  LineUart_Init();
  LineUart_InitSensor();
  LineControl_Init();
  Ps2Control_Init();
  Esp12fComm_Init();
  Esp12fFlashBridge_Init();
  Usart1DebugConsole_Init();
  SSD1306_Init();
  OLED_UI_Init();
}

uint32_t ChassisTasks_GetMissedPeriodCount(uint32_t task)
{
  return ChassisTaskTiming_GetMissedCount((chassis_task_timing_id_t)task);
}

void Task_Safety(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    SystemMonitor_Update();
    HAL_IWDG_Refresh(&hiwdg);
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_SAFETY, &next_wake, CHASSIS_ADC_PERIOD_MS);
  }
}

void Task_MotorControl(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    EncoderDriver_Update(osKernelGetTickCount());
    ChassisControl_Step(osKernelGetTickCount());
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_MOTOR, &next_wake, CHASSIS_CONTROL_PERIOD_MS);
  }
}

void Task_RpiComm(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    UpperUart_Update();
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_RPI, &next_wake, UPPER_UART_TASK_PERIOD_MS);
  }
}

void Task_Imu(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    (void)ImuBmi270_Update();
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_IMU, &next_wake, CHASSIS_IMU_PERIOD_MS);
  }
}

void Task_Line(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    LineUart_Update();
    LineUart_RequestAnalog();
    LineControl_Update();
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_LINE, &next_wake, CHASSIS_LINE_PERIOD_MS);
  }
}

void Task_Esp12f(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    Esp12fFlashBridge_Update(osKernelGetTickCount());
    Esp12fComm_Update();
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_ESP, &next_wake, CHASSIS_ESP12F_PERIOD_MS);
  }
}

void Task_Ps2(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    Ps2Control_Update();
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_PS2, &next_wake, CHASSIS_PS2_PERIOD_MS);
  }
}

void Task_Led(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    LedStatus_TaskStep(CHASSIS_LED_PERIOD_MS);
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_LED, &next_wake, CHASSIS_LED_PERIOD_MS);
  }
}

void Task_Oled(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    OLED_UI_Update();
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_OLED, &next_wake, OLED_TASK_PERIOD_MS);
  }
}
