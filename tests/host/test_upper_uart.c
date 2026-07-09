#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "chassis_control.h"
#include "chassis_layout.h"
#include "control_manager.h"
#include "encoder_driver.h"
#include "imu_bmi270.h"
#include "line_control.h"
#include "motor_driver.h"
#include "system_monitor.h"
#include "upper_uart.h"
#include "upper_protocol.h"
#include "usart.h"

static USART_TypeDef usart3_instance = {0};
static DMA_Stream_TypeDef usart3_rx_stream = {0};
static DMA_HandleTypeDef hdma_usart3_rx = { .Instance = &usart3_rx_stream };
UART_HandleTypeDef huart3 = { .Instance = &usart3_instance, .hdmarx = &hdma_usart3_rx, .gState = HAL_UART_STATE_READY };

static uint8_t *rx_buffer;
static uint16_t rx_size;
static uint32_t rx_start_count;
static uint32_t dma_stop_count;
static uint32_t tx_dma_count;
static uint32_t tx_it_count;
static uint8_t tx_last_frame[UPPER_PROTOCOL_MAX_FRAME];
static uint16_t tx_last_size;
static UART_HandleTypeDef *tx_last_uart;
static uint32_t fake_tick;

uint32_t osKernelGetTickCount(void)
{
  return fake_tick;
}

int32_t osDelayUntil(uint32_t ticks)
{
  fake_tick = ticks;
  return 0;
}

HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  if (huart == &huart3)
  {
    rx_buffer = pData;
    rx_size = Size;
    huart->hdmarx = &hdma_usart3_rx;
    huart->hdmarx->Instance->NDTR = Size;
    huart->Instance->CR3 |= USART_CR3_DMAR;
    rx_start_count++;
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
  tx_dma_count++;
  tx_last_uart = huart;
  if (pData != 0 && Size <= (uint16_t)sizeof(tx_last_frame))
  {
    for (uint16_t i = 0U; i < Size; ++i)
    {
      tx_last_frame[i] = pData[i];
    }
    tx_last_size = Size;
  }
  return HAL_BUSY;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
  tx_it_count++;
  tx_last_uart = huart;
  if (pData != 0 && Size <= (uint16_t)sizeof(tx_last_frame))
  {
    for (uint16_t i = 0U; i < Size; ++i)
    {
      tx_last_frame[i] = pData[i];
    }
    tx_last_size = Size;
  }
  huart->gState = HAL_UART_STATE_READY;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    huart->Instance->CR3 &= ~(USART_CR3_DMAR | USART_CR3_DMAT);
    dma_stop_count++;
  }
  return HAL_OK;
}

control_command_result_t ControlManager_SetCommand(const chassis_cmd_t *cmd)
{
  (void)cmd;
  return CONTROL_COMMAND_ACCEPTED;
}

void ControlManager_SetEmergencyStop(uint8_t enable)
{
  (void)enable;
}

uint8_t ControlManager_IsEmergencyStop(void)
{
  return 0U;
}

uint8_t ControlManager_IsFaultStop(void)
{
  return 0U;
}

void LineControl_Enable(uint8_t enable)
{
  (void)enable;
}

uint8_t LineControl_IsEnabled(void)
{
  return 0U;
}

void SystemMonitor_ClearLatchedFaults(uint32_t mask)
{
  (void)mask;
}

void ChassisControl_GetState(chassis_control_state_t *state)
{
  if (state != 0)
  {
    *state = (chassis_control_state_t){0};
  }
}

void EncoderDriver_GetState(encoder_state_t *state)
{
  if (state != 0)
  {
    *state = (encoder_state_t){0};
  }
}

void MotorDriver_GetState(motor_driver_state_t *state)
{
  if (state != 0)
  {
    *state = (motor_driver_state_t){0};
  }
}

void SystemMonitor_GetState(system_monitor_state_t *state)
{
  if (state != 0)
  {
    *state = (system_monitor_state_t){0};
  }
}

uint8_t ChassisLayout_MotorEnabled(motor_id_t motor)
{
  return (motor == MOTOR_ID_M2 || motor == MOTOR_ID_M3) ? 1U : 0U;
}

void ImuBmi270_GetState(imu_bmi270_state_t *state)
{
  if (state != 0)
  {
    *state = (imu_bmi270_state_t){0};
  }
}


static void reset_host_uart_state(void)
{
  rx_buffer = 0;
  rx_size = 0U;
  rx_start_count = 0U;
  dma_stop_count = 0U;
  tx_dma_count = 0U;
  tx_it_count = 0U;
  tx_last_size = 0U;
  tx_last_uart = 0;
  fake_tick = 0U;
  usart3_instance = (USART_TypeDef){0};
  usart3_rx_stream = (DMA_Stream_TypeDef){0};
  hdma_usart3_rx = (DMA_HandleTypeDef){ .Instance = &usart3_rx_stream };
  huart3 = (UART_HandleTypeDef){ .Instance = &usart3_instance, .hdmarx = &hdma_usart3_rx, .gState = HAL_UART_STATE_READY };
  for (uint16_t i = 0U; i < (uint16_t)sizeof(tx_last_frame); ++i)
  {
    tx_last_frame[i] = 0U;
  }
}
static void require_int(int condition, const char *message)
{
  if (condition == 0)
  {
    (void)printf("FAIL: %s\n", message);
    exit(1);
  }
}

static void feed_valid_estop_frame(void)
{
  uint8_t payload = 1U;
  uint8_t frame[UPPER_PROTOCOL_MAX_FRAME] = {0};
  uint16_t len = UpperProtocol_BuildFrame(UPPER_CMD_ESTOP,
                                          &payload,
                                          UPPER_PROTOCOL_ESTOP_PAYLOAD_LEN,
                                          frame,
                                          (uint16_t)sizeof(frame));
  for (uint16_t i = 0U; i < len; ++i)
  {
    rx_buffer[i] = frame[i];
  }
  huart3.hdmarx->Instance->NDTR = (uint32_t)(rx_size - len);
}

static void test_dma_callbacks_and_valid_frame_timestamp(void)
{
  upper_uart_state_t state;

  reset_host_uart_state();
  fake_tick = 1000U;
  UpperUart_Init();
  require_int(rx_start_count == 1U, "upper uart starts rx dma");

  UpperUart_OnDmaHalf();
  UpperUart_OnDmaFull();
  feed_valid_estop_frame();
  UpperUart_Update();
  UpperUart_GetState(&state);
  require_int(state.rx_dma_half_count == 1U, "rx half count");
  require_int(state.rx_dma_full_count == 1U, "rx full count");
  require_int(state.last_valid_frame_ms == 1000U, "last valid frame timestamp");
}

static void test_parser_timeout_and_uart_error_restart_dma(void)
{
  upper_uart_state_t state;
  uint32_t starts_before;

  reset_host_uart_state();
  UpperUart_Init();
  rx_buffer[0] = UPPER_PROTOCOL_HEAD_0;
  huart3.hdmarx->Instance->NDTR = (uint32_t)(rx_size - 1U);
  UpperUart_Update();
  for (uint8_t i = 0U; i < 21U; ++i)
  {
    UpperUart_Update();
  }
  UpperUart_GetState(&state);
  require_int(state.rx_timeout_resets != 0U, "parser timeout reset counted");
  require_int(state.rx_resync_restarts != 0U, "parser timeout restarts dma");

  starts_before = rx_start_count;
  huart3.Instance->SR = UART_FLAG_ORE | UART_FLAG_NE;
  UpperUart_OnUartError();
  UpperUart_GetState(&state);
  require_int(dma_stop_count != 0U, "uart error stops dma");
  require_int(rx_start_count == starts_before + 1U, "uart error restarts dma");
  require_int(state.uart_errors != 0U, "uart error counted");
  require_int(huart3.Instance->SR == 0U, "uart flags cleared");
}


static void test_status_uses_interrupt_tx_without_usart3_tx_dma(void)
{
  upper_uart_state_t state;

  reset_host_uart_state();
  fake_tick = 1000U;
  huart3.hdmatx = 0;
  huart3.gState = HAL_UART_STATE_READY;

  UpperUart_Init();
  UpperUart_Update();
  UpperUart_GetState(&state);

  require_int(tx_dma_count == 0U, "usart3 status must not use tx dma when no tx dma is configured");
  require_int(tx_it_count == 1U, "usart3 status uses interrupt tx");
  require_int(tx_last_uart == &huart3, "status frame sent on usart3");
  require_int(tx_last_size > 5U, "status frame has bytes");
  require_int(tx_last_frame[0] == UPPER_PROTOCOL_HEAD_0, "status frame head0");
  require_int(tx_last_frame[1] == UPPER_PROTOCOL_HEAD_1, "status frame head1");
  require_int(tx_last_frame[3] == UPPER_CMD_STATUS, "status frame cmd");
  require_int(state.tx_frames == 1U, "status tx frame counted");
}
int main(void)
{
  test_status_uses_interrupt_tx_without_usart3_tx_dma();
  test_dma_callbacks_and_valid_frame_timestamp();
  test_parser_timeout_and_uart_error_restart_dma();
  (void)printf("PASS: upper uart host tests\n");
  return 0;
}
