#ifndef LINE_UART_H
#define LINE_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* HiWonder 八路巡线传感器协议常量 */
#define LINE_SENSOR_CHANNELS   8U
#define LINE_SENSOR_FRAME_LEN  21U
#define LINE_SENSOR_HEADER_0   0x55U
#define LINE_SENSOR_HEADER_1   0xAAU
#define LINE_SENSOR_CMD_ANALOG 0x02U

/* 模拟量阈值：低于此值判定为检测到黑线 */
#define LINE_ANALOG_THRESHOLD  500U

    typedef struct
    {
        uint32_t rx_bytes;
        uint32_t rx_frames;
        uint32_t overflow_count;
        uint32_t rx_protocol_errors;
        uint32_t tx_frames;
        uint32_t tx_busy_drops;
        uint32_t tx_failures;
        uint32_t uart_errors;
        uint32_t dma_restarts;
        uint16_t last_frame_len;
        uint8_t  last_frame[32];
        uint8_t  tx_busy;
    } line_sensor_driver_state_t;

    typedef struct
    {
        uint8_t  state[LINE_SENSOR_CHANNELS];
        uint16_t analog[LINE_SENSOR_CHANNELS];
        uint32_t timestamp_ms;
        uint8_t  valid;
    } line_sensor_data_t;

    void    LineSensorDriver_Init(void);
    void    LineSensorDriver_SetThresholdConfig(const uint16_t threshold[LINE_SENSOR_CHANNELS], uint8_t active_low);
    void    LineSensorDriver_Update(void);
    void    LineSensorDriver_GetState(line_sensor_driver_state_t *state);
    uint8_t LineSensorDriver_GetSensorData(line_sensor_data_t *data);
    void    LineSensorDriver_InitSensor(void);
    void    LineSensorDriver_RequestAnalog(void);
    void    LineSensorDriver_OnTxCplt(void);
    void    LineSensorDriver_OnUartError(void);

#ifdef __cplusplus
}
#endif

#endif
