#ifndef ESP12F_FLASH_BRIDGE_H
#define ESP12F_FLASH_BRIDGE_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t active;
  uint8_t download_mode;
  uint32_t pc_to_esp_rx_bytes;
  uint32_t esp_to_pc_rx_bytes;
  uint32_t pc_to_esp_tx_bytes;
  uint32_t esp_to_pc_tx_bytes;
  uint32_t pc_to_esp_overflow;
  uint32_t esp_to_pc_overflow;
  uint32_t uart_error_count;
  uint32_t auto_exit_count;
  uint32_t last_auto_exit_idle_ms;
  uint32_t rx_start_errors;
  uint32_t last_activity_ms;
} esp12f_flash_bridge_state_t;

void Esp12fFlashBridge_Init(void);
uint8_t Esp12fFlashBridge_Enable(uint8_t download_mode);
void Esp12fFlashBridge_Disable(void);
uint8_t Esp12fFlashBridge_IsActive(void);
uint32_t Esp12fFlashBridge_GetIdleMs(void);
void Esp12fFlashBridge_Update(uint32_t now_ms);
void Esp12fFlashBridge_OnRxCplt(UART_HandleTypeDef *huart);
void Esp12fFlashBridge_OnTxCplt(UART_HandleTypeDef *huart);
void Esp12fFlashBridge_OnUartError(UART_HandleTypeDef *huart);
void Esp12fFlashBridge_GetState(esp12f_flash_bridge_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
