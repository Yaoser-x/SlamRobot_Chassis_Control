/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis_tasks.h"
#include "iwdg.h"
#include "reset_trace.h"

#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FREERTOS_FATAL_ASSERT          1UL
#define FREERTOS_FATAL_STACK_OVERFLOW  2UL
#define FREERTOS_FATAL_MALLOC_FAILED   3UL
#define FREERTOS_FATAL_TASK_CREATE     4UL
#define SAFETY_TASK_STACK_SIZE_BYTES   4096U
#define SAFETY_TASK_STACK_MIN_BYTES    3072U

_Static_assert(SAFETY_TASK_STACK_SIZE_BYTES >= SAFETY_TASK_STACK_MIN_BYTES,
               "safetyTask stack is below the monitored-control budget");

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile uint32_t freertos_fatal_reason;
volatile uint32_t freertos_fatal_line;
volatile const char *freertos_fatal_file;

osThreadId_t usart1DebugTaskHandle;
const osThreadAttr_t usart1DebugTask_attributes = {
  .name = "debugTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

osThreadId_t ps2TaskHandle;
const osThreadAttr_t ps2Task_attributes = {
  .name = "ps2Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for safetyTask */
osThreadId_t safetyTaskHandle;
const osThreadAttr_t safetyTask_attributes = {
  .name = "safetyTask",
  .stack_size = SAFETY_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for rpiCommTask */
osThreadId_t rpiCommTaskHandle;
const osThreadAttr_t rpiCommTask_attributes = {
  .name = "rpiCommTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lineTask */
osThreadId_t lineTaskHandle;
const osThreadAttr_t lineTask_attributes = {
  .name = "lineTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for espTask */
osThreadId_t espTaskHandle;
const osThreadAttr_t espTask_attributes = {
  .name = "espTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void FreeRtos_FatalStop(uint32_t reason,
                               const char *file,
                               uint32_t line,
                               reset_trace_task_t task);
static reset_trace_task_t FreeRtos_TaskFromName(const char *name);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);  /* safetyTask — 状态聚合/看门狗 */
void StartTask03(void *argument);  /* motorTask — 电机控制链 */
void StartTask04(void *argument);  /* rpiCommTask — USART3 上位机协议 */
void StartTask05(void *argument);  /* imuTask — BMI270 采样 */
void StartTask06(void *argument);  /* lineTask — 巡线传感器 */
void StartTask07(void *argument);  /* espTask — ESP12F 协议 */
void Task_Usart1DebugConsole(void *argument);
void Task_Ps2(void *argument);
void Task_Led(void *argument);
void Task_Oled(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
static reset_trace_task_t FreeRtos_TaskFromName(const char *name)
{
  if (name == NULL)
  {
    return RESET_TRACE_TASK_NONE;
  }
  if (strcmp(name, "safetyTask") == 0)
  {
    return RESET_TRACE_TASK_SAFETY;
  }
  if (strcmp(name, "motorTask") == 0)
  {
    return RESET_TRACE_TASK_MOTOR;
  }
  if (strcmp(name, "ps2Task") == 0)
  {
    return RESET_TRACE_TASK_PS2;
  }
  if (strcmp(name, "espTask") == 0)
  {
    return RESET_TRACE_TASK_ESP;
  }
  if (strcmp(name, "debugTask") == 0)
  {
    return RESET_TRACE_TASK_DEBUG;
  }
  return RESET_TRACE_TASK_NONE;
}

static void FreeRtos_FatalStop(uint32_t reason,
                               const char *file,
                               uint32_t line,
                               reset_trace_task_t task)
{
  freertos_fatal_reason = reason;
  freertos_fatal_file = file;
  freertos_fatal_line = line;
  ResetTrace_CaptureWithTask(RESET_TRACE_KIND_FREERTOS, reason, line, task);
  DRV_SLEEP_ALL_GPIO_Port->BSRR = ((uint32_t)DRV_SLEEP_ALL_Pin << 16U);
  __disable_irq();
  for (;;)
  {
    __NOP();
  }
}

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
  (void)xTask;
  FreeRtos_FatalStop(FREERTOS_FATAL_STACK_OVERFLOW,
                     (const char *)pcTaskName,
                     0U,
                     FreeRtos_TaskFromName((const char *)pcTaskName));
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
  demo application. If heap_1.c or heap_2.c are used, then the size of the
  heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
  FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
  to query the size of free heap space that remains (although it does not
  provide information on how the remaining heap might be fragmented). */
  FreeRtos_FatalStop(FREERTOS_FATAL_MALLOC_FAILED, 0, 0U, RESET_TRACE_TASK_NONE);
}
/* USER CODE END 5 */

/* USER CODE BEGIN 6 */
void vApplicationAssertHook(const char *file, unsigned long line)
{
  FreeRtos_FatalStop(FREERTOS_FATAL_ASSERT, file, (uint32_t)line, RESET_TRACE_TASK_NONE);
}
/* USER CODE END 6 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  ChassisTasks_InitHardware();

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of safetyTask */
  safetyTaskHandle = osThreadNew(StartTask02, NULL, &safetyTask_attributes);

  /* creation of motorTask */
  motorTaskHandle = osThreadNew(StartTask03, NULL, &motorTask_attributes);

  /* creation of rpiCommTask */
  rpiCommTaskHandle = osThreadNew(StartTask04, NULL, &rpiCommTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(StartTask05, NULL, &imuTask_attributes);

  /* creation of lineTask */
  lineTaskHandle = osThreadNew(StartTask06, NULL, &lineTask_attributes);

  /* creation of espTask */
  espTaskHandle = osThreadNew(StartTask07, NULL, &espTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  usart1DebugTaskHandle = osThreadNew(Task_Usart1DebugConsole, NULL, &usart1DebugTask_attributes);
  ps2TaskHandle = osThreadNew(Task_Ps2, NULL, &ps2Task_attributes);
  ledTaskHandle = osThreadNew(Task_Led, NULL, &ledTask_attributes);
  oledTaskHandle = osThreadNew(Task_Oled, NULL, &oledTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  if (defaultTaskHandle == NULL ||
      safetyTaskHandle == NULL ||
      motorTaskHandle == NULL ||
      rpiCommTaskHandle == NULL ||
      imuTaskHandle == NULL ||
      lineTaskHandle == NULL ||
      espTaskHandle == NULL ||
      usart1DebugTaskHandle == NULL ||
      ps2TaskHandle == NULL ||
      ledTaskHandle == NULL ||
      oledTaskHandle == NULL)
  {
    FreeRtos_FatalStop(FREERTOS_FATAL_TASK_CREATE, __FILE__, __LINE__, RESET_TRACE_TASK_NONE);
  }
#if defined(DEBUG)
  __HAL_DBGMCU_FREEZE_IWDG();
#endif
  MX_IWDG_Init();
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the safetyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  Task_Safety(argument);
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  Task_MotorControl(argument);
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the rpiCommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  Task_RpiComm(argument);
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  Task_Imu(argument);
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the lineTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  Task_Line(argument);
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the espTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  Task_Esp12f(argument);
  /* USER CODE END StartTask07 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
