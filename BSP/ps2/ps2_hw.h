#ifndef PS2_HW_H
#define PS2_HW_H

/**
 * @file ps2_hw.h
 * @brief PS2 手柄位带硬件驱动。
 *
 * 提供 DWT 微秒延时、SPI 位带传输、手柄初始化和数据读取。
 * 上层通过 Ps2Hw_ReadSample() 获取手柄原始状态。
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PS2_HW_FRAME_LEN  9U

typedef struct
{
  uint8_t mode;
  uint8_t btn1;
  uint8_t btn2;
  uint8_t right_x;
  uint8_t right_y;
  uint8_t left_x;
  uint8_t left_y;
} ps2_hw_sample_t;

/**
 * @brief 初始化 DWT 延时、GPIO 引脚和手柄模拟模式。
 */
void Ps2Hw_Init(void);

/**
 * @brief 读取一帧手柄数据。
 * @param sample 输出参数，填充手柄状态。
 * @return 1 成功，0 通信失败。
 */
uint8_t Ps2Hw_ReadSample(ps2_hw_sample_t *sample);

/**
 * @brief 判断给定模式字节是否为模拟模式。
 */
uint8_t Ps2Hw_IsAnalogMode(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* PS2_HW_H */
