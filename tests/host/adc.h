#ifndef TEST_ADC_H
#define TEST_ADC_H

#include <stdint.h>

/* --------------------------------------------------------------------------
 * Minimal HAL stubs sufficient to compile adc_monitor.c on a host machine.
 * Values match STM32F4 HAL headers but the structs are stripped down.  */

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

#define HAL_MAX_DELAY  0xFFFFFFFFU

/* DMA register stub — only CR is touched by __HAL_DMA_DISABLE_IT */
typedef struct
{
  uint32_t CR;
} DMA_Stream_TypeDef;

typedef struct
{
  DMA_Stream_TypeDef *Instance;
} DMA_HandleTypeDef;

#define DMA_IT_TC  ((uint32_t)0x00000010U)  /* DMA_SxCR_TCIE */
#define DMA_IT_HT  ((uint32_t)0x00000008U)  /* DMA_SxCR_HTIE */
#define DMA_IT_FE  ((uint32_t)0x00000080U)

extern uint32_t host_dma_disabled_interrupt_mask;

#define __HAL_DMA_DISABLE_IT(__HANDLE__, __INTERRUPT__)  \
  do { (void)(__HANDLE__); host_dma_disabled_interrupt_mask |= (__INTERRUPT__); } while (0)

typedef struct
{
  DMA_HandleTypeDef DMA_Handle;
} ADC_HandleTypeDef;

extern ADC_HandleTypeDef hadc1;

HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *buffer, uint32_t length);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t primask);

#endif
