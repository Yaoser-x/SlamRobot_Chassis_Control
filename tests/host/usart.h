#ifndef HOST_USART_H
#define HOST_USART_H

#include <stdint.h>

#include "main.h"

typedef struct
{
  uint32_t NDTR;
} DMA_Stream_TypeDef;

typedef struct
{
  DMA_Stream_TypeDef *Instance;
} DMA_HandleTypeDef;

typedef struct
{
  uint32_t CR3;
  uint32_t SR;
} USART_TypeDef;

typedef enum
{
  HAL_UART_STATE_RESET   = 0x00U,
  HAL_UART_STATE_READY   = 0x20U,
  HAL_UART_STATE_BUSY    = 0x24U,
  HAL_UART_STATE_BUSY_TX = 0x21U,
  HAL_UART_STATE_BUSY_RX = 0x22U
} HAL_UART_StateTypeDef;

typedef struct
{
  USART_TypeDef *Instance;
  DMA_HandleTypeDef *hdmarx;
  DMA_HandleTypeDef *hdmatx;
  HAL_UART_StateTypeDef gState;
  HAL_UART_StateTypeDef RxState;
  uint32_t ErrorCode;
} UART_HandleTypeDef;

#define USART_CR3_DMAR 0x00000040U
#define USART_CR3_DMAT 0x00000080U
#define UART_FLAG_ORE  0x00000008U
#define UART_FLAG_NE   0x00000004U
#define UART_FLAG_FE   0x00000002U
#define UART_FLAG_PE   0x00000001U

#define __HAL_DMA_GET_COUNTER(__HANDLE__) ((__HANDLE__)->Instance->NDTR)
#define __HAL_UART_CLEAR_OREFLAG(__HANDLE__) ((__HANDLE__)->Instance->SR &= ~UART_FLAG_ORE)
#define __HAL_UART_CLEAR_NEFLAG(__HANDLE__)   ((__HANDLE__)->Instance->SR &= ~UART_FLAG_NE)
#define __HAL_UART_CLEAR_FEFLAG(__HANDLE__)   ((__HANDLE__)->Instance->SR &= ~UART_FLAG_FE)
#define __HAL_UART_CLEAR_PEFLAG(__HANDLE__)   ((__HANDLE__)->Instance->SR &= ~UART_FLAG_PE)

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;

HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart);

#endif
