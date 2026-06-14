#ifndef LINE_UART_H
#define LINE_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* HiWonder 八路巡线传感器协议常量 */
#define LINE_SENSOR_CHANNELS     8U
#define LINE_SENSOR_FRAME_LEN    21U
#define LINE_SENSOR_HEADER_0     0x55U
#define LINE_SENSOR_HEADER_1     0xAAU
#define LINE_SENSOR_CMD_ANALOG   0x02U

/* 模拟量阈值：低于此值判定为检测到黑线 */
#define LINE_ANALOG_THRESHOLD    500U

typedef struct
{
  uint32_t rx_bytes;
  uint32_t rx_frames;
  uint32_t overflow_count;
  uint32_t rx_protocol_errors;
  uint16_t last_frame_len;
  uint8_t  last_frame[32];
} line_uart_state_t;

typedef struct
{
  uint8_t  state[LINE_SENSOR_CHANNELS];
  uint16_t analog[LINE_SENSOR_CHANNELS];
  uint32_t timestamp_ms;
  uint8_t  valid;
} line_sensor_data_t;

void     LineUart_Init(void);
void     LineUart_Update(void);
void     LineUart_GetState(line_uart_state_t *state);
uint8_t  LineUart_GetSensorData(line_sensor_data_t *data);
void     LineUart_InitSensor(void);
void     LineUart_RequestAnalog(void);

#ifdef __cplusplus
}
#endif

#endif
