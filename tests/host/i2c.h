#ifndef HOST_I2C_H
#define HOST_I2C_H

#include <stdint.h>

#include "main.h"

typedef struct
{
    uint32_t CR1;
} I2C_TypeDef;

typedef struct
{
    I2C_TypeDef *Instance;
} I2C_HandleTypeDef;

#define I2C_MEMADD_SIZE_8BIT 1U
#define I2C_CR1_PE           0x0001U

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c,
                                    uint16_t           DevAddress,
                                    uint16_t           MemAddress,
                                    uint16_t           MemAddSize,
                                    uint8_t           *pData,
                                    uint16_t           Size,
                                    uint32_t           Timeout);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);

#endif
