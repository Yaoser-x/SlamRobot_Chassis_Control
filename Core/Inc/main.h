/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PS2_DI_Pin GPIO_PIN_2
#define PS2_DI_GPIO_Port GPIOE
#define PS2_DO_Pin GPIO_PIN_3
#define PS2_DO_GPIO_Port GPIOE
#define PS2_CS_Pin GPIO_PIN_4
#define PS2_CS_GPIO_Port GPIOE
#define PS2_CLK_Pin GPIO_PIN_5
#define PS2_CLK_GPIO_Port GPIOE
#define TEST_LED_Pin GPIO_PIN_6
#define TEST_LED_GPIO_Port GPIOE
#define M1_CURRENT_Pin GPIO_PIN_0
#define M1_CURRENT_GPIO_Port GPIOC
#define M2_CURRENT_Pin GPIO_PIN_1
#define M2_CURRENT_GPIO_Port GPIOC
#define M3_CURRENT_Pin GPIO_PIN_2
#define M3_CURRENT_GPIO_Port GPIOC
#define M4_CURRENT_Pin GPIO_PIN_3
#define M4_CURRENT_GPIO_Port GPIOC
#define M4_ENC_A_Pin GPIO_PIN_0
#define M4_ENC_A_GPIO_Port GPIOA
#define M4_ENC_B_Pin GPIO_PIN_1
#define M4_ENC_B_GPIO_Port GPIOA
#define M1_FAULT_Pin GPIO_PIN_2
#define M1_FAULT_GPIO_Port GPIOA
#define M2_FAULT_Pin GPIO_PIN_3
#define M2_FAULT_GPIO_Port GPIOA
#define VBAT_SENSE_Pin GPIO_PIN_4
#define VBAT_SENSE_GPIO_Port GPIOC
#define DRV_SLEEP_ALL_Pin GPIO_PIN_7
#define DRV_SLEEP_ALL_GPIO_Port GPIOE
#define M1_IN1_Pin GPIO_PIN_9
#define M1_IN1_GPIO_Port GPIOE
#define M2_IN1_Pin GPIO_PIN_11
#define M2_IN1_GPIO_Port GPIOE
#define M3_IN1_Pin GPIO_PIN_13
#define M3_IN1_GPIO_Port GPIOE
#define M4_IN1_Pin GPIO_PIN_14
#define M4_IN1_GPIO_Port GPIOE
#define TIM1_BKIN_Pin GPIO_PIN_15
#define TIM1_BKIN_GPIO_Port GPIOE
#define ESP_EN_Pin GPIO_PIN_10
#define ESP_EN_GPIO_Port GPIOB
#define ESP_RST_Pin GPIO_PIN_11
#define ESP_RST_GPIO_Port GPIOB
#define IMU_CS_Pin GPIO_PIN_12
#define IMU_CS_GPIO_Port GPIOB
#define IMU_SCK_Pin GPIO_PIN_13
#define IMU_SCK_GPIO_Port GPIOB
#define IMU_MISO_Pin GPIO_PIN_14
#define IMU_MISO_GPIO_Port GPIOB
#define IMU_MOSI_Pin GPIO_PIN_15
#define IMU_MOSI_GPIO_Port GPIOB
#define RPI_TX_Pin GPIO_PIN_8
#define RPI_TX_GPIO_Port GPIOD
#define RPI_RX_Pin GPIO_PIN_9
#define RPI_RX_GPIO_Port GPIOD
#define M3_ENC_A_Pin GPIO_PIN_12
#define M3_ENC_A_GPIO_Port GPIOD
#define M3_ENC_B_Pin GPIO_PIN_13
#define M3_ENC_B_GPIO_Port GPIOD
#define M3_FAULT_Pin GPIO_PIN_14
#define M3_FAULT_GPIO_Port GPIOD
#define M4_FAULT_Pin GPIO_PIN_15
#define M4_FAULT_GPIO_Port GPIOD
#define M1_IN2_Pin GPIO_PIN_6
#define M1_IN2_GPIO_Port GPIOC
#define M2_IN2_Pin GPIO_PIN_7
#define M2_IN2_GPIO_Port GPIOC
#define M3_IN2_Pin GPIO_PIN_8
#define M3_IN2_GPIO_Port GPIOC
#define M4_IN2_Pin GPIO_PIN_9
#define M4_IN2_GPIO_Port GPIOC
#define M1_ENC_A_Pin GPIO_PIN_15
#define M1_ENC_A_GPIO_Port GPIOA
#define LINE_TX_Pin GPIO_PIN_10
#define LINE_TX_GPIO_Port GPIOC
#define LINE_RX_Pin GPIO_PIN_11
#define LINE_RX_GPIO_Port GPIOC
#define ESP_TX_Pin GPIO_PIN_5
#define ESP_TX_GPIO_Port GPIOD
#define ESP_RX_Pin GPIO_PIN_6
#define ESP_RX_GPIO_Port GPIOD
#define ESP_IO0_Pin GPIO_PIN_7
#define ESP_IO0_GPIO_Port GPIOD
#define M1_ENC_B_Pin GPIO_PIN_3
#define M1_ENC_B_GPIO_Port GPIOB
#define M2_ENC_A_Pin GPIO_PIN_4
#define M2_ENC_A_GPIO_Port GPIOB
#define M2_ENC_B_Pin GPIO_PIN_5
#define M2_ENC_B_GPIO_Port GPIOB
#define DEBUG_TX_Pin GPIO_PIN_6
#define DEBUG_TX_GPIO_Port GPIOB
#define DEBUG_RX_Pin GPIO_PIN_7
#define DEBUG_RX_GPIO_Port GPIOB
#define IMU_INT1_Pin GPIO_PIN_0
#define IMU_INT1_GPIO_Port GPIOE
#define IMU_INT1_EXTI_IRQn EXTI0_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
