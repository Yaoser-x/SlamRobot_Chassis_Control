#ifndef HOST_MAIN_H
#define HOST_MAIN_H

#include <stdint.h>

typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET = 1
} GPIO_PinState;

typedef struct
{
  uint32_t id;
} GPIO_TypeDef;

typedef struct
{
  uint32_t ARR;
  uint32_t CCR1;
  uint32_t CCR2;
  uint32_t CCR3;
  uint32_t CCR4;
  uint32_t SR;
  uint32_t DIER;
  uint32_t BDTR;
} TIM_TypeDef;

typedef struct
{
  TIM_TypeDef *Instance;
} TIM_HandleTypeDef;

#ifndef HOST_HAL_STATUS_TYPEDEF
#define HOST_HAL_STATUS_TYPEDEF
typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;
#endif

#define RESET 0U

#define GPIO_PIN_0  ((uint16_t)0x0001)
#define GPIO_PIN_1  ((uint16_t)0x0002)
#define GPIO_PIN_2  ((uint16_t)0x0004)
#define GPIO_PIN_3  ((uint16_t)0x0008)
#define GPIO_PIN_6  ((uint16_t)0x0040)
#define GPIO_PIN_7  ((uint16_t)0x0080)
#define GPIO_PIN_8  ((uint16_t)0x0100)
#define GPIO_PIN_9  ((uint16_t)0x0200)
#define GPIO_PIN_14 ((uint16_t)0x4000)
#define GPIO_PIN_15 ((uint16_t)0x8000)

#define TIM_CHANNEL_1 0x00000000U
#define TIM_CHANNEL_2 0x00000004U
#define TIM_CHANNEL_3 0x00000008U
#define TIM_CHANNEL_4 0x0000000CU
#define TIM_FLAG_BREAK 0x00000080U
#define TIM_IT_BREAK 0x00000080U
#define TIM_BDTR_MOE 0x00008000U
#define TIM1_BKIN_Pin GPIO_PIN_15
#define TIM1_BKIN_GPIO_Port GPIOE

extern GPIO_TypeDef GPIOA_Instance;
extern GPIO_TypeDef GPIOC_Instance;
extern GPIO_TypeDef GPIOD_Instance;
extern GPIO_TypeDef GPIOE_Instance;

#define GPIOA (&GPIOA_Instance)
#define GPIOC (&GPIOC_Instance)
#define GPIOD (&GPIOD_Instance)
#define GPIOE (&GPIOE_Instance)

#define M1_FAULT_Pin GPIO_PIN_2
#define M1_FAULT_GPIO_Port GPIOA
#define M2_FAULT_Pin GPIO_PIN_3
#define M2_FAULT_GPIO_Port GPIOA
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
#define DRV_SLEEP_ALL_Pin GPIO_PIN_7
#define DRV_SLEEP_ALL_GPIO_Port GPIOE

uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t primask);
void HostTimSetCompare(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t pulse);
uint32_t HostTimGetCompare(TIM_HandleTypeDef *htim, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_Delay(uint32_t delay_ms);

#define __HAL_TIM_GET_AUTORELOAD(htim) ((htim)->Instance->ARR)
#define __HAL_TIM_SET_COMPARE(htim, channel, pulse) HostTimSetCompare((htim), (channel), (pulse))
#define __HAL_TIM_GET_FLAG(htim, flag) ((((htim)->Instance->SR & (flag)) != 0U) ? 1U : 0U)
#define __HAL_TIM_CLEAR_FLAG(htim, flag) ((htim)->Instance->SR &= ~(flag))
#define __HAL_TIM_ENABLE_IT(htim, it) ((htim)->Instance->DIER |= (it))

#endif
