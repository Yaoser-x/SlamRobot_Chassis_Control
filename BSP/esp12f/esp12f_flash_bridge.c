#include "esp12f_flash_bridge.h"

#include "chassis_maintenance.h"
#include "chassis_control.h"
#include "control_manager.h"
#include "esp12f_comm.h"
#include "main.h"
#include "usart.h"
#include "usart1_debug_console.h"

#define ESP12F_FLASH_BRIDGE_RING_SIZE       4096U
#define ESP12F_FLASH_BRIDGE_TX_CHUNK_SIZE   128U
#define ESP12F_FLASH_BRIDGE_IDLE_TIMEOUT_MS 30000U
#define ESP12F_FLASH_BRIDGE_EN_LOW_MS       50U
#define ESP12F_FLASH_BRIDGE_EN_TO_RST_MS    10U
#define ESP12F_FLASH_BRIDGE_BOOT_WAIT_MS    100U

typedef struct
{
    uint8_t           data[ESP12F_FLASH_BRIDGE_RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} bridge_ring_t;

static bridge_ring_t               pc_to_esp_ring;
static bridge_ring_t               esp_to_pc_ring;
static uint8_t                     pc_to_esp_tx_chunk[ESP12F_FLASH_BRIDGE_TX_CHUNK_SIZE];
static uint8_t                     esp_to_pc_tx_chunk[ESP12F_FLASH_BRIDGE_TX_CHUNK_SIZE];
static uint8_t                     usart1_rx_byte;
static uint8_t                     usart2_rx_byte;
static volatile uint8_t            pc_to_esp_tx_busy;
static volatile uint8_t            esp_to_pc_tx_busy;
static esp12f_flash_bridge_state_t bridge_state;
static uint8_t                     bridge_maintenance_lock_held;

static uint32_t Esp12fFlashBridge_GetIdleMsLocked(void)
{
    uint32_t now_ms;
    uint32_t last_activity_ms;
    uint32_t primask;
    int32_t  elapsed_ms;

    primask = __get_PRIMASK();
    __disable_irq();
    now_ms           = HAL_GetTick();
    last_activity_ms = bridge_state.last_activity_ms;
    __set_PRIMASK(primask);

    elapsed_ms = (int32_t)(now_ms - last_activity_ms);
    if (elapsed_ms <= 0)
    {
        return 0U;
    }

    return (uint32_t)elapsed_ms;
}

/* BridgeRing_ 前缀：本文件内部环形缓冲区操作，与模块级命名空间隔离 */
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
    ring->head             = next_head;
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
        ring->tail   = (uint16_t)((ring->tail + 1U) % ESP12F_FLASH_BRIDGE_RING_SIZE);
    }
    __set_PRIMASK(primask);

    return count;
}

static void Esp12fFlashBridge_ResetTxBusy(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    pc_to_esp_tx_busy = 0U;
    esp_to_pc_tx_busy = 0U;
    __set_PRIMASK(primask);
}

static void Esp12fFlashBridge_StartPcToEspTx(void)
{
    uint16_t len = 0U;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    if (pc_to_esp_tx_busy == 0U)
    {
        len = BridgeRing_PopChunk(&pc_to_esp_ring, pc_to_esp_tx_chunk, (uint16_t)sizeof(pc_to_esp_tx_chunk));
        if (len > 0U)
        {
            pc_to_esp_tx_busy = 1U;
        }
    }
    __set_PRIMASK(primask);

    if (len > 0U)
    {
        if (HAL_UART_Transmit_IT(&huart2, pc_to_esp_tx_chunk, len) == HAL_OK)
        {
            bridge_state.pc_to_esp_tx_bytes += len;
        }
        else
        {
            pc_to_esp_tx_busy = 0U;
            bridge_state.uart_error_count++;
        }
    }
}

static void Esp12fFlashBridge_StartEspToPcTx(void)
{
    uint16_t len = 0U;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    if (esp_to_pc_tx_busy == 0U)
    {
        len = BridgeRing_PopChunk(&esp_to_pc_ring, esp_to_pc_tx_chunk, (uint16_t)sizeof(esp_to_pc_tx_chunk));
        if (len > 0U)
        {
            esp_to_pc_tx_busy = 1U;
        }
    }
    __set_PRIMASK(primask);

    if (len > 0U)
    {
        if (HAL_UART_Transmit_IT(&huart1, esp_to_pc_tx_chunk, len) == HAL_OK)
        {
            bridge_state.esp_to_pc_tx_bytes += len;
        }
        else
        {
            esp_to_pc_tx_busy = 0U;
            bridge_state.uart_error_count++;
        }
    }
}

static void Esp12fFlashBridge_StartDownloadBoot(void)
{
    /* 1. Pull EN LOW for full chip power-on reset (not just RST) */
    HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ESP_IO0_GPIO_Port, ESP_IO0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(ESP12F_FLASH_BRIDGE_EN_LOW_MS);

    /* 2. Release EN - ESP8266 samples IO0 on EN rising edge */
    HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(ESP12F_FLASH_BRIDGE_EN_TO_RST_MS);

    /* 3. Release RST */
    HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(ESP12F_FLASH_BRIDGE_BOOT_WAIT_MS);
}

static void Esp12fFlashBridge_StartNormalBoot(void)
{
    /* 1. Pull EN LOW for full chip power-on reset */
    HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ESP_IO0_GPIO_Port, ESP_IO0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(ESP12F_FLASH_BRIDGE_EN_LOW_MS);

    /* 2. Release EN - ESP8266 samples IO0 on EN rising edge */
    HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(ESP12F_FLASH_BRIDGE_EN_TO_RST_MS);

    /* 3. Release RST */
    HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(ESP12F_FLASH_BRIDGE_BOOT_WAIT_MS);
}

static void Esp12fFlashBridge_ResetRuntimeState(uint32_t now_ms)
{
    BridgeRing_Clear(&pc_to_esp_ring);
    BridgeRing_Clear(&esp_to_pc_ring);
    bridge_state.pc_to_esp_rx_bytes     = 0U;
    bridge_state.esp_to_pc_rx_bytes     = 0U;
    bridge_state.pc_to_esp_tx_bytes     = 0U;
    bridge_state.esp_to_pc_tx_bytes     = 0U;
    bridge_state.pc_to_esp_overflow     = 0U;
    bridge_state.esp_to_pc_overflow     = 0U;
    bridge_state.uart_error_count       = 0U;
    bridge_state.last_auto_exit_idle_ms = 0U;
    bridge_state.rx_start_errors        = 0U;
    bridge_state.last_activity_ms       = now_ms;
}

void Esp12fFlashBridge_Init(void)
{
    bridge_state                 = (esp12f_flash_bridge_state_t){0};
    bridge_maintenance_lock_held = 0U;
    BridgeRing_Clear(&pc_to_esp_ring);
    BridgeRing_Clear(&esp_to_pc_ring);
    Esp12fFlashBridge_ResetTxBusy();
}

uint8_t Esp12fFlashBridge_Enable(uint8_t download_mode)
{
    uint32_t now_ms = HAL_GetTick();
    uint8_t  rx_ok  = 1U;

    if (bridge_state.active != 0U)
    {
        return 1U;
    }
    if (ChassisMaintenance_Begin() != CHASSIS_MAINTENANCE_OK)
    {
        return 0U;
    }
    bridge_maintenance_lock_held = 1U;

    ControlManager_ClearCommand();
    Usart1DebugConsole_RevokeMaintenanceAuthorization();
    ChassisControl_CancelTestMode();

    (void)HAL_UART_Abort(&huart1);
    (void)HAL_UART_Abort(&huart2);
    Esp12fFlashBridge_ResetTxBusy();
    Usart1DebugConsole_ClearRxBuffer();

    /* Clear any lingering UART error flags (ORE/FE/NE/PE) that could block
   * subsequent HAL_UART_Receive_IT calls.  HAL_UART_Abort does not clear
   * these flags, and a stale ORE would prevent RXNE interrupts from firing. */
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart2);

    Esp12fFlashBridge_ResetRuntimeState(now_ms);
    bridge_state.active        = 1U;
    bridge_state.download_mode = (download_mode != 0U) ? 1U : 0U;

    if (download_mode != 0U)
    {
        Esp12fFlashBridge_StartDownloadBoot();
    }
    else
    {
        Esp12fFlashBridge_StartNormalBoot();
    }

    if (HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1U) != HAL_OK)
    {
        bridge_state.rx_start_errors++;
        rx_ok = 0U;
    }
    if (HAL_UART_Receive_IT(&huart2, &usart2_rx_byte, 1U) != HAL_OK)
    {
        bridge_state.rx_start_errors++;
        rx_ok = 0U;
    }

    if (rx_ok == 0U)
    {
        (void)HAL_UART_Abort(&huart1);
        (void)HAL_UART_Abort(&huart2);
        Esp12fFlashBridge_ResetTxBusy();
        BridgeRing_Clear(&pc_to_esp_ring);
        BridgeRing_Clear(&esp_to_pc_ring);
        bridge_state.active        = 0U;
        bridge_state.download_mode = 0U;
        Esp12fFlashBridge_StartNormalBoot();
        Usart1DebugConsole_RestartRx();
        Esp12fComm_RestartRx();
        bridge_maintenance_lock_held = 0U;
        ChassisMaintenance_End();
        return 0U;
    }

    return 1U;
}

void Esp12fFlashBridge_Disable(void)
{
    if (bridge_state.active == 0U)
    {
        Esp12fComm_SetDownloadMode(0U);
        if (bridge_maintenance_lock_held != 0U)
        {
            bridge_maintenance_lock_held = 0U;
            ChassisMaintenance_End();
        }
        return;
    }

    (void)HAL_UART_Abort(&huart1);
    (void)HAL_UART_Abort(&huart2);
    Esp12fFlashBridge_ResetTxBusy();
    BridgeRing_Clear(&pc_to_esp_ring);
    BridgeRing_Clear(&esp_to_pc_ring);
    bridge_state.active        = 0U;
    bridge_state.download_mode = 0U;

    Esp12fFlashBridge_StartNormalBoot();
    Usart1DebugConsole_RestartRx();
    Esp12fComm_RestartRx();
    bridge_maintenance_lock_held = 0U;
    ChassisMaintenance_End();
}

uint8_t Esp12fFlashBridge_IsActive(void)
{
    uint8_t  active;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    active = bridge_state.active;
    __set_PRIMASK(primask);
    return active;
}

uint32_t Esp12fFlashBridge_GetIdleMs(void)
{
    return Esp12fFlashBridge_GetIdleMsLocked();
}

void Esp12fFlashBridge_Update(uint32_t now_ms)
{
    uint32_t idle_ms;

    (void)now_ms;

    if (bridge_state.active == 0U)
    {
        return;
    }

    Esp12fFlashBridge_StartPcToEspTx();
    Esp12fFlashBridge_StartEspToPcTx();

    idle_ms = Esp12fFlashBridge_GetIdleMsLocked();
    if (idle_ms >= ESP12F_FLASH_BRIDGE_IDLE_TIMEOUT_MS)
    {
        bridge_state.last_auto_exit_idle_ms = idle_ms;
        bridge_state.auto_exit_count++;
        Esp12fFlashBridge_Disable();
    }
}

void Esp12fFlashBridge_OnTxCplt(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        esp_to_pc_tx_busy = 0U;
        Esp12fFlashBridge_StartEspToPcTx();
    }
    else if (huart == &huart2)
    {
        pc_to_esp_tx_busy = 0U;
        Esp12fFlashBridge_StartPcToEspTx();
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
    if (huart == &huart1)
    {
        esp_to_pc_tx_busy = 0U;
        (void)HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1U);
    }
    else if (huart == &huart2)
    {
        pc_to_esp_tx_busy = 0U;
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
