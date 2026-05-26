#include "esp12f_flash_bridge.h"

#include "chassis_control.h"
#include "control_manager.h"
#include "esp12f_comm.h"
#include "main.h"
#include "usart.h"
#include "usart1_debug_console.h"

#define ESP12F_FLASH_BRIDGE_RING_SIZE       1024U
#define ESP12F_FLASH_BRIDGE_TX_CHUNK_SIZE   64U
#define ESP12F_FLASH_BRIDGE_UART_TIMEOUT_MS 20U
#define ESP12F_FLASH_BRIDGE_IDLE_TIMEOUT_MS 30000U
#define ESP12F_FLASH_BRIDGE_RESET_LOW_MS    20U
#define ESP12F_FLASH_BRIDGE_BOOT_WAIT_MS    50U

typedef struct
{
  uint8_t data[ESP12F_FLASH_BRIDGE_RING_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
} bridge_ring_t;

static bridge_ring_t pc_to_esp_ring;
static bridge_ring_t esp_to_pc_ring;
static uint8_t usart1_rx_byte;
static uint8_t usart2_rx_byte;
static esp12f_flash_bridge_state_t bridge_state;

static void BridgeRing_Clear(bridge_ring_t *ring)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  ring->head = 0U;
  ring->tail = 0U;
  __set_PRIMASK(primask);
}

static uint8_t BridgeRing_Push(bridge_ring_t *ring, uint8_t byte)
{
  uint16_t next_head = (uint16_t)((ring->head + 1U) % ESP12F_FLASH_BRIDGE_RING_SIZE);

  if (next_head == ring->tail)
  {
    return 0U;
  }

  ring->data[ring->head] = byte;
  ring->head = next_head;
  return 1U;
}

static uint16_t BridgeRing_PopChunk(bridge_ring_t *ring, uint8_t *out, uint16_t max_len)
{
  uint16_t count = 0U;
  uint32_t primask;

  if (out == 0 || max_len == 0U)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  while (ring->tail != ring->head && count < max_len)
  {
    out[count++] = ring->data[ring->tail];
    ring->tail = (uint16_t)((ring->tail + 1U) % ESP12F_FLASH_BRIDGE_RING_SIZE);
  }
  __set_PRIMASK(primask);

  return count;
}

static void Esp12fFlashBridge_StartDownloadBoot(void)
{
  HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ESP_IO0_GPIO_Port, ESP_IO0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(ESP12F_FLASH_BRIDGE_RESET_LOW_MS);
  HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(ESP12F_FLASH_BRIDGE_BOOT_WAIT_MS);
}

static void Esp12fFlashBridge_StartNormalBoot(void)
{
  HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ESP_IO0_GPIO_Port, ESP_IO0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(ESP12F_FLASH_BRIDGE_RESET_LOW_MS);
  HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(ESP12F_FLASH_BRIDGE_BOOT_WAIT_MS);
}

static void Esp12fFlashBridge_ResetRuntimeState(uint32_t now_ms)
{
  BridgeRing_Clear(&pc_to_esp_ring);
  BridgeRing_Clear(&esp_to_pc_ring);
  bridge_state.pc_to_esp_rx_bytes = 0U;
  bridge_state.esp_to_pc_rx_bytes = 0U;
  bridge_state.pc_to_esp_tx_bytes = 0U;
  bridge_state.esp_to_pc_tx_bytes = 0U;
  bridge_state.pc_to_esp_overflow = 0U;
  bridge_state.esp_to_pc_overflow = 0U;
  bridge_state.uart_error_count = 0U;
  bridge_state.last_activity_ms = now_ms;
}

void Esp12fFlashBridge_Init(void)
{
  bridge_state = (esp12f_flash_bridge_state_t){0};
  BridgeRing_Clear(&pc_to_esp_ring);
  BridgeRing_Clear(&esp_to_pc_ring);
}

void Esp12fFlashBridge_Enable(void)
{
  uint32_t now_ms = HAL_GetTick();

  if (bridge_state.active != 0U)
  {
    return;
  }

  ControlManager_ClearCommand();
  ChassisControl_OpenLoopTest(0, 0);
  ChassisControl_RawInputTest(0, 0, 0, 0);

  (void)HAL_UART_Abort(&huart1);
  (void)HAL_UART_Abort(&huart2);
  Esp12fFlashBridge_ResetRuntimeState(now_ms);
  bridge_state.active = 1U;
  bridge_state.download_mode = 1U;

  Esp12fFlashBridge_StartDownloadBoot();
  (void)HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1U);
  (void)HAL_UART_Receive_IT(&huart2, &usart2_rx_byte, 1U);
}

void Esp12fFlashBridge_Disable(void)
{
  if (bridge_state.active == 0U)
  {
    Esp12fComm_SetDownloadMode(0U);
    return;
  }

  (void)HAL_UART_Abort(&huart1);
  (void)HAL_UART_Abort(&huart2);
  BridgeRing_Clear(&pc_to_esp_ring);
  BridgeRing_Clear(&esp_to_pc_ring);
  bridge_state.active = 0U;
  bridge_state.download_mode = 0U;

  Esp12fFlashBridge_StartNormalBoot();
  Usart1DebugConsole_RestartRx();
  Esp12fComm_RestartRx();
}

uint8_t Esp12fFlashBridge_IsActive(void)
{
  uint8_t active;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  active = bridge_state.active;
  __set_PRIMASK(primask);
  return active;
}

void Esp12fFlashBridge_Update(uint32_t now_ms)
{
  uint8_t tx_chunk[ESP12F_FLASH_BRIDGE_TX_CHUNK_SIZE];
  uint16_t len;

  if (bridge_state.active == 0U)
  {
    return;
  }

  len = BridgeRing_PopChunk(&pc_to_esp_ring, tx_chunk, (uint16_t)sizeof(tx_chunk));
  if (len > 0U)
  {
    if (HAL_UART_Transmit(&huart2, tx_chunk, len, ESP12F_FLASH_BRIDGE_UART_TIMEOUT_MS) == HAL_OK)
    {
      bridge_state.pc_to_esp_tx_bytes += len;
    }
    else
    {
      bridge_state.uart_error_count++;
    }
  }

  len = BridgeRing_PopChunk(&esp_to_pc_ring, tx_chunk, (uint16_t)sizeof(tx_chunk));
  if (len > 0U)
  {
    if (HAL_UART_Transmit(&huart1, tx_chunk, len, ESP12F_FLASH_BRIDGE_UART_TIMEOUT_MS) == HAL_OK)
    {
      bridge_state.esp_to_pc_tx_bytes += len;
    }
    else
    {
      bridge_state.uart_error_count++;
    }
  }

  if ((now_ms - bridge_state.last_activity_ms) >= ESP12F_FLASH_BRIDGE_IDLE_TIMEOUT_MS)
  {
    bridge_state.auto_exit_count++;
    Esp12fFlashBridge_Disable();
  }
}

void Esp12fFlashBridge_OnRxCplt(UART_HandleTypeDef *huart)
{
  if (bridge_state.active == 0U)
  {
    return;
  }

  bridge_state.last_activity_ms = HAL_GetTick();
  if (huart == &huart1)
  {
    if (BridgeRing_Push(&pc_to_esp_ring, usart1_rx_byte) != 0U)
    {
      bridge_state.pc_to_esp_rx_bytes++;
    }
    else
    {
      bridge_state.pc_to_esp_overflow++;
    }
    (void)HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1U);
  }
  else if (huart == &huart2)
  {
    if (BridgeRing_Push(&esp_to_pc_ring, usart2_rx_byte) != 0U)
    {
      bridge_state.esp_to_pc_rx_bytes++;
    }
    else
    {
      bridge_state.esp_to_pc_overflow++;
    }
    (void)HAL_UART_Receive_IT(&huart2, &usart2_rx_byte, 1U);
  }
}

void Esp12fFlashBridge_OnUartError(UART_HandleTypeDef *huart)
{
  if (bridge_state.active == 0U)
  {
    return;
  }

  bridge_state.uart_error_count++;
  bridge_state.last_activity_ms = HAL_GetTick();
  if (huart == &huart1)
  {
    (void)HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1U);
  }
  else if (huart == &huart2)
  {
    (void)HAL_UART_Receive_IT(&huart2, &usart2_rx_byte, 1U);
  }
}

void Esp12fFlashBridge_GetState(esp12f_flash_bridge_state_t *state)
{
  if (state != 0)
  {
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    *state = bridge_state;
    __set_PRIMASK(primask);
  }
}
