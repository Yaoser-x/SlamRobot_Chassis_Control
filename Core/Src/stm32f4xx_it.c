/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_driver.h"
#include "reset_trace.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_DMA_GUARD_REASON_INSTANCE (1UL << 0)
#define ADC_DMA_GUARD_REASON_BASE     (1UL << 1)
#define ADC_DMA_GUARD_REASON_INDEX    (1UL << 2)
#define ADC_DMA_GUARD_REASON_PARENT   (1UL << 3)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern DMA_HandleTypeDef hdma_adc1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void HardFault_HandlerC(uint32_t *stack, uint32_t exc_return) __attribute__((used, noreturn));
static void Fault_HandlerC(uint32_t *stack, uint32_t exc_return, reset_trace_kind_t kind) __attribute__((used, noreturn));
static uint32_t AdcDmaGuardReason(void);
static void AdcDmaGuardStop(uint32_t reason);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Fault_DisableMotorDriver(void)
{
  DRV_SLEEP_ALL_GPIO_Port->BSRR = ((uint32_t)DRV_SLEEP_ALL_Pin << 16U);
}

static void HardFault_HandlerC(uint32_t *stack, uint32_t exc_return)
{
  ResetTrace_CaptureFaultStack(RESET_TRACE_KIND_HARDFAULT, stack, exc_return);
  Fault_DisableMotorDriver();
  while (1)
  {
  }
}

static void Fault_HandlerC(uint32_t *stack, uint32_t exc_return, reset_trace_kind_t kind)
{
  ResetTrace_CaptureFaultStack(kind, stack, exc_return);
  Fault_DisableMotorDriver();
  while (1)
  {
  }
}

static uint32_t AdcDmaGuardReason(void)
{
  uint32_t reason = 0U;

  if (hdma_adc1.Instance != DMA2_Stream0)
  {
    reason |= ADC_DMA_GUARD_REASON_INSTANCE;
  }
  if (hdma_adc1.StreamBaseAddress != (uint32_t)DMA2)
  {
    reason |= ADC_DMA_GUARD_REASON_BASE;
  }
  if (hdma_adc1.StreamIndex != 0U)
  {
    reason |= ADC_DMA_GUARD_REASON_INDEX;
  }
  if (hdma_adc1.Parent == 0)
  {
    reason |= ADC_DMA_GUARD_REASON_PARENT;
  }
  return reason;
}

static void AdcDmaGuardStop(uint32_t reason)
{
  ResetTrace_CaptureWithDetails(RESET_TRACE_KIND_DMA_GUARD,
                                reason,
                                __LINE__,
                                RESET_TRACE_TASK_NONE,
                                (uint32_t)hdma_adc1.Instance,
                                hdma_adc1.StreamBaseAddress,
                                hdma_adc1.StreamIndex,
                                (uint32_t)hdma_adc1.Parent);
  DMA2->LIFCR = 0x3FU;
  HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
  Fault_DisableMotorDriver();
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  ResetTrace_Capture(RESET_TRACE_KIND_NMI, 0U, 0U);
  Fault_DisableMotorDriver();

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
__attribute__((naked)) void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  __asm volatile
  (
    "tst lr, #4      \n"
    "ite eq          \n"
    "mrseq r0, msp   \n"
    "mrsne r0, psp   \n"
    "mov r1, lr      \n"
    "b HardFault_HandlerC \n"
  );
  /* USER CODE END HardFault_IRQn 0 */
}

/**
  * @brief This function handles Memory management fault.
  */
__attribute__((naked)) void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  __asm volatile
  (
    "tst lr, #4      \n"
    "ite eq          \n"
    "mrseq r0, msp   \n"
    "mrsne r0, psp   \n"
    "mov r1, lr      \n"
    "mov r2, %0      \n"
    "b Fault_HandlerC \n"
    :
    : "i" (RESET_TRACE_KIND_MEMMANAGE)
  );
  /* USER CODE END MemoryManagement_IRQn 0 */
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
__attribute__((naked)) void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  __asm volatile
  (
    "tst lr, #4      \n"
    "ite eq          \n"
    "mrseq r0, msp   \n"
    "mrsne r0, psp   \n"
    "mov r1, lr      \n"
    "mov r2, %0      \n"
    "b Fault_HandlerC \n"
    :
    : "i" (RESET_TRACE_KIND_BUSFAULT)
  );
  /* USER CODE END BusFault_IRQn 0 */
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
__attribute__((naked)) void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  __asm volatile
  (
    "tst lr, #4      \n"
    "ite eq          \n"
    "mrseq r0, msp   \n"
    "mrsne r0, psp   \n"
    "mov r1, lr      \n"
    "mov r2, %0      \n"
    "b Fault_HandlerC \n"
    :
    : "i" (RESET_TRACE_KIND_USAGEFAULT)
  );
  /* USER CODE END UsageFault_IRQn 0 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  MotorDriver_OnTim1BreakFromIsr();
}

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(IMU_INT1_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  /*
   * USART2 uses interrupt-driven TX/RX only. Keep stale or corrupted DMA
   * state from sending HAL_UART_IRQHandler() into an invalid DMA abort path.
   */
  CLEAR_BIT(huart2.Instance->CR3, USART_CR3_DMAR | USART_CR3_DMAT);
  huart2.hdmarx = 0;
  huart2.hdmatx = 0;
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */
  uint32_t guard_reason = AdcDmaGuardReason();
  if (guard_reason != 0U)
  {
    AdcDmaGuardStop(guard_reason);
    return;
  }
  HAL_DMA_IRQHandler(&hdma_adc1);

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
