#include "line_uart.h"

#include "usart.h"

#define LINE_UART_RX_BUFFER_SIZE 128U

static uint8_t line_rx_dma_buffer[LINE_UART_RX_BUFFER_SIZE];
static uint16_t line_rx_read_pos;
static line_uart_state_t line_state;
static uint8_t frame_buf[sizeof(line_state.last_frame)];
static uint16_t frame_len;

void LineUart_Init(void)
{
  line_rx_read_pos = 0U;
  frame_len = 0U;
  line_state = (line_uart_state_t){0};
  (void)HAL_UART_Receive_DMA(&huart4, line_rx_dma_buffer, LINE_UART_RX_BUFFER_SIZE);
}

static void LineUart_ProcessByte(uint8_t byte)
{
  line_state.rx_bytes++;
  if (byte == '\n' || byte == '\r')
  {
    if (frame_len > 0U)
    {
      uint16_t copy_len = frame_len;
      if (copy_len > sizeof(line_state.last_frame))
      {
        copy_len = sizeof(line_state.last_frame);
      }
      for (uint16_t i = 0U; i < copy_len; ++i)
      {
        line_state.last_frame[i] = frame_buf[i];
      }
      line_state.last_frame_len = copy_len;
      line_state.rx_frames++;
      frame_len = 0U;
    }
    return;
  }

  if (frame_len < sizeof(frame_buf))
  {
    frame_buf[frame_len++] = byte;
  }
  else
  {
    line_state.overflow_count++;
    frame_len = 0U;
  }
}

void LineUart_Update(void)
{
  uint16_t write_pos;

  if (huart4.hdmarx == 0)
  {
    return;
  }

  write_pos = (uint16_t)(LINE_UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart4.hdmarx));
  if (write_pos >= LINE_UART_RX_BUFFER_SIZE)
  {
    write_pos = 0U;
  }

  while (line_rx_read_pos != write_pos)
  {
    LineUart_ProcessByte(line_rx_dma_buffer[line_rx_read_pos]);
    line_rx_read_pos++;
    if (line_rx_read_pos >= LINE_UART_RX_BUFFER_SIZE)
    {
      line_rx_read_pos = 0U;
    }
  }
}

void LineUart_GetState(line_uart_state_t *state)
{
  if (state != 0)
  {
    *state = line_state;
  }
}
