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
#include "reset_trace.h"
#include "system_monitor.h"
#include "upper_uart.h"
#include "usart1_debug_console.h"
#include "ssd1306.h"
#include "oled_ui.h"
#include "control_manager.h"
#include "main.h"

extern osThreadId_t imuTaskHandle;

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
    uint32_t now_ms = osKernelGetTickCount();

    SystemMonitor_Update();
    ResetTrace_UpdateControl(ControlManager_GetActiveSource(),
                             ControlManager_IsEmergencyStop(),
                             ControlManager_IsFaultStop());
    ResetTrace_TaskHeartbeat(RESET_TRACE_TASK_SAFETY, now_ms);

    /* 看门狗守卫：仅当电机任务心跳在 200ms 内更新时才喂狗。
       电机任务挂死 → 心跳过期 → IWDG 超时复位。 */
    {
      uint32_t motor_hb = ResetTrace_GetTaskHeartbeat(RESET_TRACE_TASK_MOTOR);
      if (motor_hb != 0U && (now_ms - motor_hb) <= 200U)
      {
        HAL_IWDG_Refresh(&hiwdg);
      }
    }
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_SAFETY, &next_wake, CHASSIS_ADC_PERIOD_MS);
  }
}

void Task_MotorControl(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    uint32_t now_ms = osKernelGetTickCount();

    ResetTrace_TaskHeartbeat(RESET_TRACE_TASK_MOTOR, now_ms);
    EncoderDriver_Update(now_ms);
    ChassisControl_Step(now_ms);
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
  (void)argument;
  for (;;)
  {
    (void)osThreadFlagsWait(CHASSIS_IMU_TASK_FLAG_DRDY,
                            osFlagsWaitAny,
                            CHASSIS_IMU_PERIOD_MS);
    (void)ImuBmi270_Update();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == IMU_INT1_Pin)
  {
    ImuBmi270_OnDataReadyFromIsr();
    if (imuTaskHandle != NULL)
    {
      (void)osThreadFlagsSet(imuTaskHandle, CHASSIS_IMU_TASK_FLAG_DRDY);
    }
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
    uint32_t now_ms = osKernelGetTickCount();

    ResetTrace_TaskHeartbeat(RESET_TRACE_TASK_ESP, now_ms);
    Esp12fFlashBridge_Update(now_ms);
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
    ResetTrace_TaskHeartbeat(RESET_TRACE_TASK_PS2, osKernelGetTickCount());
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
