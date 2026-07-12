#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "line_uart.h"
#include "param_store.h"
#include "usart.h"

static USART_TypeDef uart4_instance = {0};
static DMA_Stream_TypeDef uart4_rx_stream = {0};
static DMA_HandleTypeDef hdma_uart4_rx = { .Instance = &uart4_rx_stream };
UART_HandleTypeDef huart4 = { .Instance = &uart4_instance, .hdmarx = &hdma_uart4_rx, .gState = HAL_UART_STATE_READY };

static uint32_t fake_tick;
static uint8_t *rx_buffer;
static uint16_t rx_size;
static uint32_t rx_start_count;
static uint32_t dma_stop_count;
static HAL_StatusTypeDef next_tx_status = HAL_OK;
static uint8_t last_tx_byte;
static uint32_t tx_it_count;

uint32_t __get_PRIMASK(void) { return 0U; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t value) { (void)value; }

uint32_t osKernelGetTickCount(void)
{
  return fake_tick;
}

HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  if (huart == &huart4)
  {
    rx_buffer = pData;
    rx_size = Size;
    huart->hdmarx = &hdma_uart4_rx;
    huart->RxState = HAL_UART_STATE_BUSY_RX;
    huart->Instance->CR3 |= USART_CR3_DMAR;
    huart->hdmarx->Instance->NDTR = Size;
    rx_start_count++;
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
  if (huart != &huart4 || pData == 0 || Size != 1U)
  {
    return HAL_ERROR;
  }
  if (next_tx_status == HAL_OK)
  {
    huart->gState = HAL_UART_STATE_BUSY_TX;
    last_tx_byte = pData[0];
    tx_it_count++;
  }
  return next_tx_status;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
  return HAL_UART_Transmit_IT(huart, pData, Size);
}

HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart)
{
  if (huart == &huart4)
  {
    huart->Instance->CR3 &= ~(USART_CR3_DMAR | USART_CR3_DMAT);
    dma_stop_count++;
  }
  return HAL_OK;
}

static void require_int(int condition, const char *message)
{
  if (condition == 0)
  {
    (void)printf("FAIL: %s\n", message);
    exit(1);
  }
}

static void test_nonblocking_tx_busy_and_complete(void)
{
  line_uart_state_t state;

  LineUart_Init();
  require_int(rx_start_count == 1U, "rx dma starts");

  LineUart_InitSensor();
  LineUart_GetState(&state);
  require_int(tx_it_count == 1U, "init sensor uses interrupt tx");
  require_int(last_tx_byte == 0x00U, "init sensor byte");
  require_int(state.tx_busy != 0U, "tx busy after async start");
  require_int(state.tx_frames == 1U, "tx frame count after init");

  LineUart_RequestAnalog();
  LineUart_GetState(&state);
  require_int(tx_it_count == 1U, "busy request does not start another tx");
  require_int(state.tx_busy_drops == 1U, "busy drop counted");

  LineUart_OnTxCplt();
  LineUart_RequestAnalog();
  LineUart_GetState(&state);
  require_int(tx_it_count == 2U, "request starts after complete");
  require_int(last_tx_byte == LINE_SENSOR_CMD_ANALOG, "analog query byte");
  require_int(state.tx_frames == 2U, "tx frame count after query");
}

static void test_tx_failure_and_uart_error_restart_dma(void)
{
  line_uart_state_t state;
  uint32_t rx_before;

  LineUart_OnTxCplt();
  next_tx_status = HAL_ERROR;
  LineUart_RequestAnalog();
  LineUart_GetState(&state);
  require_int(state.tx_failures == 1U, "tx failure counted");
  require_int(state.tx_busy == 0U, "tx failure leaves tx idle");
  next_tx_status = HAL_OK;

  rx_before = rx_start_count;
  huart4.Instance->SR = UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE;
  LineUart_OnUartError();
  LineUart_GetState(&state);
  require_int(dma_stop_count == 1U, "uart error stops dma");
  require_int(rx_start_count == rx_before + 1U, "uart error restarts dma");
  require_int(state.uart_errors == 1U, "uart error counted");
  require_int(state.dma_restarts == 1U, "dma restart counted");
  require_int(huart4.Instance->SR == 0U, "uart error flags cleared");
}

static void test_rx_frame_still_parses_after_restart(void)
{
  line_sensor_data_t data;
  uint8_t frame[LINE_SENSOR_FRAME_LEN] = {0x55U, 0xAAU, LINE_SENSOR_CMD_ANALOG, 0x10U};
  uint16_t checksum_sum = 0U;

  for (uint8_t ch = 0U; ch < LINE_SENSOR_CHANNELS; ++ch)
  {
    uint16_t raw = (uint16_t)(100U + ch);
    frame[4U + (ch * 2U)] = (uint8_t)(raw & 0xFFU);
    frame[5U + (ch * 2U)] = (uint8_t)(raw >> 8);
  }
  for (uint8_t i = 2U; i < 20U; ++i)
  {
    checksum_sum += frame[i];
  }
  frame[20] = (uint8_t)(~checksum_sum);

  for (uint8_t i = 0U; i < LINE_SENSOR_FRAME_LEN; ++i)
  {
    rx_buffer[i] = frame[i];
  }
  fake_tick = 1234U;
  huart4.hdmarx->Instance->NDTR = (uint32_t)(rx_size - LINE_SENSOR_FRAME_LEN);
  LineUart_Update();
  require_int(LineUart_GetSensorData(&data) != 0U, "line frame valid");
  require_int(data.timestamp_ms == 1234U, "line timestamp");
  require_int(data.analog[7] == 107U, "line analog channel");

  {
    param_store_t params;
    ParamStore_Get(&params);
    params.line_active_low = 0U;
    for (uint8_t ch = 0U; ch < LINE_SENSOR_CHANNELS; ++ch)
    {
      uint16_t raw = (ch & 1U) ? 100U : 600U;
      params.line_threshold_raw[ch] = 500U;
      frame[4U + (ch * 2U)] = (uint8_t)(raw & 0xFFU);
      frame[5U + (ch * 2U)] = (uint8_t)(raw >> 8);
    }
    require_int(ParamStore_Set(&params) != 0U, "active-high polarity accepted");
    checksum_sum = 0U;
    for (uint8_t i = 2U; i < 20U; ++i) { checksum_sum += frame[i]; }
    frame[20] = (uint8_t)(~checksum_sum);
    for (uint8_t i = 0U; i < LINE_SENSOR_FRAME_LEN; ++i)
    {
      rx_buffer[LINE_SENSOR_FRAME_LEN + i] = frame[i];
    }
    fake_tick++;
    huart4.hdmarx->Instance->NDTR = (uint32_t)(rx_size - (2U * LINE_SENSOR_FRAME_LEN));
    LineUart_Update();
    (void)LineUart_GetSensorData(&data);
    require_int(data.state[0] == 1U && data.state[1] == 0U,
                "runtime active-high polarity is applied in one mapping layer");
  }
}

int main(void)
{
  test_nonblocking_tx_busy_and_complete();
  test_tx_failure_and_uart_error_restart_dma();
  test_rx_frame_still_parses_after_restart();
  (void)printf("PASS: line uart host tests\n");
  return 0;
}
