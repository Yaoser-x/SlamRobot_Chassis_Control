#include "line_uart.h"

#include "platform_time.h"
#include "usart.h"

#define LINE_UART_RX_BUFFER_SIZE 128U

/* DMA 循环缓冲 */
static uint8_t  line_rx_dma_buffer[LINE_UART_RX_BUFFER_SIZE];
static uint16_t line_rx_read_pos;

/* 监控/调试状态 */
static line_uart_state_t line_state;

/* 协议帧解析器 */
typedef enum
{
    LINE_RX_WAIT_HEAD0 = 0,
    LINE_RX_WAIT_HEAD1,
    LINE_RX_WAIT_CMD,
    LINE_RX_WAIT_DATA_LEN,
    LINE_RX_WAIT_DATA,
    LINE_RX_WAIT_CHECKSUM
} line_rx_state_t;

static line_rx_state_t line_rx_state;
static uint8_t         line_frame_buf[LINE_SENSOR_FRAME_LEN];
static uint8_t         line_frame_index;
static uint8_t         line_data_len;
static uint8_t         line_data_idx;
static uint8_t         line_tx_busy;

/* 最近一次成功解析的传感器数据 */
static line_sensor_data_t line_sensor_data;
static const uint8_t      line_manual_mode_cmd                 = 0x00U;
static const uint8_t      line_analog_query_cmd                = LINE_SENSOR_CMD_ANALOG;
static uint16_t           line_threshold[LINE_SENSOR_CHANNELS] = {500U, 500U, 500U, 500U, 500U, 500U, 500U, 500U};
static uint8_t            line_active_low                      = 1U;

void LineUart_SetThresholdConfig(const uint16_t threshold[LINE_SENSOR_CHANNELS], uint8_t active_low)
{
    if (threshold == 0)
    {
        return;
    }
    for (uint8_t i = 0U; i < LINE_SENSOR_CHANNELS; ++i)
    {
        line_threshold[i] = threshold[i];
    }
    line_active_low = (active_low != 0U) ? 1U : 0U;
}

/* ---------- 校验和 ---------- */

static uint8_t LineUart_ComputeChecksum(const uint8_t *buf, uint8_t data_len)
{
    uint16_t sum = 0U;
    uint8_t  end = (uint8_t)(2U + 1U + data_len + 1U); /* cmd + len + data + 1 = data_len + 4 */

    for (uint8_t i = 2U; i < end; ++i)
    {
        sum += buf[i];
    }
    return (uint8_t)(~sum);
}

static void LineUart_ResetParser(void)
{
    line_rx_state    = LINE_RX_WAIT_HEAD0;
    line_frame_index = 0U;
    line_data_len    = 0U;
    line_data_idx    = 0U;
}

static void LineUart_ClearUartFlags(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart4);
    __HAL_UART_CLEAR_NEFLAG(&huart4);
    __HAL_UART_CLEAR_FEFLAG(&huart4);
    __HAL_UART_CLEAR_PEFLAG(&huart4);
}

static void LineUart_RestartRxDma(uint8_t count_restart)
{
    (void)HAL_UART_DMAStop(&huart4);
    LineUart_ClearUartFlags();
    line_rx_read_pos = 0U;
    LineUart_ResetParser();
    (void)HAL_UART_Receive_DMA(&huart4, line_rx_dma_buffer, LINE_UART_RX_BUFFER_SIZE);
    if (count_restart != 0U)
    {
        line_state.dma_restarts++;
    }
}

static void LineUart_SendByteAsync(const uint8_t *cmd)
{
    if (line_tx_busy != 0U)
    {
        line_state.tx_busy_drops++;
        return;
    }

    line_tx_busy       = 1U;
    line_state.tx_busy = 1U;
    if (HAL_UART_Transmit_IT(&huart4, (uint8_t *)cmd, 1U) == HAL_OK)
    {
        line_state.tx_frames++;
    }
    else
    {
        line_tx_busy       = 0U;
        line_state.tx_busy = 0U;
        line_state.tx_failures++;
    }
}

/* ---------- 帧解析 ---------- */

static void LineUart_ProcessByte(uint8_t byte)
{
    line_state.rx_bytes++;

    switch (line_rx_state)
    {
        case LINE_RX_WAIT_HEAD0:
            if (byte == LINE_SENSOR_HEADER_0)
            {
                line_frame_buf[0] = byte;
                line_frame_index  = 1U;
                line_rx_state     = LINE_RX_WAIT_HEAD1;
            }
            break;

        case LINE_RX_WAIT_HEAD1:
            if (byte == LINE_SENSOR_HEADER_1)
            {
                line_frame_buf[1] = byte;
                line_frame_index  = 2U;
                line_rx_state     = LINE_RX_WAIT_CMD;
            }
            else if (byte != LINE_SENSOR_HEADER_0)
            {
                line_rx_state = LINE_RX_WAIT_HEAD0;
            }
            /* else: byte == 0x55, stay in WAIT_HEAD1 (re-sync on overlapping header) */
            break;

        case LINE_RX_WAIT_CMD:
            line_frame_buf[2] = byte;
            line_frame_index  = 3U;
            line_rx_state     = LINE_RX_WAIT_DATA_LEN;
            break;

        case LINE_RX_WAIT_DATA_LEN:
            line_frame_buf[3] = byte;
            line_data_len     = byte;
            if (line_data_len > (LINE_SENSOR_FRAME_LEN - 5U))
            {
                /* 长度字段非法，丢弃并重新同步 */
                line_state.rx_protocol_errors++;
                line_rx_state = LINE_RX_WAIT_HEAD0;
            }
            else
            {
                line_data_idx    = 0U;
                line_frame_index = 4U;
                line_rx_state    = (line_data_len > 0U) ? LINE_RX_WAIT_DATA : LINE_RX_WAIT_CHECKSUM;
            }
            break;

        case LINE_RX_WAIT_DATA:
            line_frame_buf[line_frame_index++] = byte;
            line_data_idx++;
            if (line_data_idx >= line_data_len)
            {
                line_rx_state = LINE_RX_WAIT_CHECKSUM;
            }
            break;

        case LINE_RX_WAIT_CHECKSUM:
        {
            uint8_t received_checksum = byte;
            uint8_t expected          = LineUart_ComputeChecksum(line_frame_buf, line_data_len);

            if (received_checksum == expected)
            {
                uint8_t cmd = line_frame_buf[2];

                if (cmd == LINE_SENSOR_CMD_ANALOG && line_data_len >= 16U)
                {
                    /* 提取 8 通道数据 */
                    for (uint8_t ch = 0U; ch < LINE_SENSOR_CHANNELS; ++ch)
                    {
                        uint8_t  base = (uint8_t)(4U + ch * 2U);
                        uint16_t raw  = (uint16_t)line_frame_buf[base] | ((uint16_t)line_frame_buf[base + 1U] << 8);

                        line_sensor_data.analog[ch] = raw;
                        line_sensor_data.state[ch]  = (line_active_low != 0U) ? ((raw < line_threshold[ch]) ? 1U : 0U)
                                                                              : ((raw > line_threshold[ch]) ? 1U : 0U);
                    }
                    line_sensor_data.timestamp_ms = PlatformTime_TaskNowMs();
                    line_sensor_data.valid        = 1U;
                }

                line_state.rx_frames++;
                line_state.last_frame_len = (uint16_t)(4U + line_data_len + 1U);
                if (line_state.last_frame_len > sizeof(line_state.last_frame))
                {
                    line_state.last_frame_len = (uint16_t)sizeof(line_state.last_frame);
                }
                for (uint16_t i = 0U; i < line_state.last_frame_len; ++i)
                {
                    line_state.last_frame[i] = line_frame_buf[i];
                }
            }
            else
            {
                line_state.rx_protocol_errors++;
            }
            line_rx_state = LINE_RX_WAIT_HEAD0;
            break;
        }

        default:
            line_rx_state = LINE_RX_WAIT_HEAD0;
            break;
    }
}

/* ---------- 公开 API ---------- */

void LineUart_Init(void)
{
    line_rx_read_pos = 0U;
    line_tx_busy     = 0U;
    LineUart_ResetParser();
    line_state       = (line_uart_state_t){0};
    line_sensor_data = (line_sensor_data_t){0};
    (void)HAL_UART_Receive_DMA(&huart4, line_rx_dma_buffer, LINE_UART_RX_BUFFER_SIZE);
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

uint8_t LineUart_GetSensorData(line_sensor_data_t *data)
{
    if (data != 0)
    {
        *data = line_sensor_data;
        return data->valid;
    }
    return 0U;
}

void LineUart_InitSensor(void)
{
    /*
   * 设置传感器为手动模式（Mode 0），由主机周期性查询。
   * 协议：上电后发送单字节 0x00 进入手动模式，
   *       之后用 LineUart_RequestAnalog() 发送 0x02 请求 21 字节模拟量帧。
   * 参考：HiWonder Arduino UART 例程
   */
    LineUart_SendByteAsync(&line_manual_mode_cmd);
}

void LineUart_RequestAnalog(void)
{
    /* 发送 0x02 请求模拟量帧，传感器返回 21 字节: 0x55 0xAA 0x02 0x10 CH1..CH8 CHECKSUM */
    LineUart_SendByteAsync(&line_analog_query_cmd);
}

void LineUart_OnTxCplt(void)
{
    line_tx_busy       = 0U;
    line_state.tx_busy = 0U;
}

void LineUart_OnUartError(void)
{
    line_state.uart_errors++;
    LineUart_OnTxCplt();
    LineUart_RestartRxDma(1U);
}
