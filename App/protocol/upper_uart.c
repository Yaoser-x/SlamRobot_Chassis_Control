#include "upper_uart.h"

#include "chassis_config.h"
#include "chassis_control.h"
#include "cmsis_os2.h"
#include "control_manager.h"
#include "encoder_driver.h"
#include "imu_bmi270.h"
#include "system_monitor.h"
#include "upper_protocol.h"
#include "usart.h"

#define UPPER_UART_RX_BUFFER_SIZE 128U

typedef enum
{
  UPPER_RX_WAIT_HEAD0 = 0,
  UPPER_RX_WAIT_HEAD1,
  UPPER_RX_WAIT_LEN,
  UPPER_RX_WAIT_BODY
} upper_rx_state_t;

static uint8_t upper_rx_dma_buffer[UPPER_UART_RX_BUFFER_SIZE];
static uint16_t upper_rx_read_pos;
static upper_rx_state_t upper_rx_state;
static uint8_t upper_frame_buf[UPPER_PROTOCOL_MAX_PAYLOAD + 3U];
static uint8_t upper_frame_len;
static uint8_t upper_frame_index;
static uint8_t upper_tx_frame[UPPER_PROTOCOL_MAX_FRAME];
static uint8_t upper_status_payload[UPPER_PROTOCOL_STATUS_PAYLOAD_LEN];
static uint32_t upper_last_status_ms;
static uint32_t upper_last_rx_timestamp_ms;
static upper_uart_state_t upper_state;

static void UpperUart_ResetParser(void)
{
  upper_rx_state = UPPER_RX_WAIT_HEAD0;
  upper_frame_len = 0U;
  upper_frame_index = 0U;
}

static void UpperUart_HandleFrame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
  if (cmd == UPPER_CMD_SET_VELOCITY)
  {
    upper_velocity_payload_t velocity;
    if (UpperProtocol_ParseVelocityPayload(payload, payload_len, &velocity) != 0U)
    {
      chassis_cmd_t chassis_cmd = {
        .linear_x = velocity.linear_x,
        .angular_z = velocity.angular_z,
        .enable = velocity.enable,
        .source = CONTROL_SOURCE_UPPER,
        .timestamp_ms = osKernelGetTickCount(),
      };
      (void)velocity.mode;
      ControlManager_SetCommand(&chassis_cmd);
    }
  }
  else if (cmd == UPPER_CMD_ESTOP && payload_len == UPPER_PROTOCOL_ESTOP_PAYLOAD_LEN)
  {
    ControlManager_SetEmergencyStop(payload[0]);
  }
}

static void UpperUart_ProcessByte(uint8_t byte)
{
  switch (upper_rx_state)
  {
    case UPPER_RX_WAIT_HEAD0:
      if (byte == UPPER_PROTOCOL_HEAD_0)
      {
        upper_rx_state = UPPER_RX_WAIT_HEAD1;
      }
      break;

    case UPPER_RX_WAIT_HEAD1:
      upper_rx_state = (byte == UPPER_PROTOCOL_HEAD_1) ? UPPER_RX_WAIT_LEN : UPPER_RX_WAIT_HEAD0;
      break;

    case UPPER_RX_WAIT_LEN:
      if (byte == 0U || byte > UPPER_PROTOCOL_CMD_LEN(UPPER_PROTOCOL_MAX_PAYLOAD))
      {
        UpperUart_ResetParser();
      }
      else
      {
        upper_frame_buf[0] = byte;
        upper_frame_len = byte;
        upper_frame_index = 0U;
        upper_rx_state = UPPER_RX_WAIT_BODY;
      }
      break;

    case UPPER_RX_WAIT_BODY:
      upper_frame_index++;
      upper_frame_buf[upper_frame_index] = byte;
      if (upper_frame_index >= (uint8_t)(upper_frame_len + 1U))
      {
        uint8_t checksum = upper_frame_buf[upper_frame_index];
        uint8_t expect = UpperProtocol_Checksum8(upper_frame_buf, (uint16_t)upper_frame_len + 1U);
        if (checksum == expect)
        {
          uint8_t cmd = upper_frame_buf[1];
          const uint8_t *payload = &upper_frame_buf[2];
          uint8_t payload_len = (uint8_t)(upper_frame_len - 1U);
          UpperUart_HandleFrame(cmd, payload, payload_len);
          /* Record RX timestamp for OLED module online detection */
          upper_last_rx_timestamp_ms = osKernelGetTickCount();
        }
        UpperUart_ResetParser();
      }
      break;

    default:
      UpperUart_ResetParser();
      break;
  }
}

static void UpperUart_PollRx(void)
{
  uint16_t write_pos;

  if (huart3.hdmarx == 0)
  {
    return;
  }

  write_pos = (uint16_t)(UPPER_UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx));
  if (write_pos >= UPPER_UART_RX_BUFFER_SIZE)
  {
    write_pos = 0U;
  }

  while (upper_rx_read_pos != write_pos)
  {
    UpperUart_ProcessByte(upper_rx_dma_buffer[upper_rx_read_pos]);
    upper_rx_read_pos++;
    if (upper_rx_read_pos >= UPPER_UART_RX_BUFFER_SIZE)
    {
      upper_rx_read_pos = 0U;
    }
  }
}

static void UpperUart_SendStatus(uint32_t now_ms)
{
  upper_status_payload_t status = {0};
  chassis_control_state_t chassis_state;
  encoder_state_t encoder_state;
  system_monitor_state_t monitor_state;
  imu_bmi270_state_t imu_state;
  uint8_t payload_len;
  uint16_t frame_len;

  if ((now_ms - upper_last_status_ms) < UPPER_UART_STATUS_PERIOD_MS)
  {
    return;
  }
  upper_last_status_ms = now_ms;
  if (huart3.gState != HAL_UART_STATE_READY)
  {
    upper_state.tx_busy_drops++;
    return;
  }

  ChassisControl_GetState(&chassis_state);
  EncoderDriver_GetState(&encoder_state);
  SystemMonitor_GetState(&monitor_state);
  ImuBmi270_GetState(&imu_state);

  status.left_speed = chassis_state.left_actual_mps;
  status.right_speed = chassis_state.right_actual_mps;
  status.left_encoder = encoder_state.left_count;
  status.right_encoder = encoder_state.right_count;
  status.battery_voltage = monitor_state.battery_voltage;
  status.left_current = monitor_state.left_current_a;
  status.right_current = monitor_state.right_current_a;
  status.imu_accel[0] = imu_state.accel_raw[0];
  status.imu_accel[1] = imu_state.accel_raw[1];
  status.imu_accel[2] = imu_state.accel_raw[2];
  status.imu_gyro[0] = imu_state.gyro_raw[0];
  status.imu_gyro[1] = imu_state.gyro_raw[1];
  status.imu_gyro[2] = imu_state.gyro_raw[2];
  status.error_flags = monitor_state.error_flags;
  status.control_mode = monitor_state.control_mode;

  payload_len = UpperProtocol_BuildStatusPayload(&status, upper_status_payload, sizeof(upper_status_payload));
  frame_len = UpperProtocol_BuildFrame(UPPER_CMD_STATUS, upper_status_payload, payload_len, upper_tx_frame, sizeof(upper_tx_frame));
  if (frame_len > 0U)
  {
    if (HAL_UART_Transmit_DMA(&huart3, upper_tx_frame, frame_len) == HAL_OK)
    {
      upper_state.tx_frames++;
    }
    else
    {
      upper_state.tx_busy_drops++;
    }
  }
}

void UpperUart_Init(void)
{
  upper_rx_read_pos = 0U;
  upper_last_status_ms = 0U;
  upper_state = (upper_uart_state_t){0};
  upper_last_rx_timestamp_ms = 0U;
  UpperUart_ResetParser();
  (void)HAL_UART_Receive_DMA(&huart3, upper_rx_dma_buffer, UPPER_UART_RX_BUFFER_SIZE);
}

void UpperUart_Update(void)
{
  uint32_t now_ms = osKernelGetTickCount();

  UpperUart_PollRx();
  UpperUart_SendStatus(now_ms);
}

void Task_UpperUart(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();

  (void)argument;
  for (;;)
  {
    UpperUart_Update();
    next_wake += UPPER_UART_TASK_PERIOD_MS;
    (void)osDelayUntil(next_wake);
  }
}

void UpperUart_GetState(upper_uart_state_t *state)
{
  if (state != 0)
  {
    *state = upper_state;
  }
}

uint32_t UpperUart_GetLastRxTimestamp(void)
{
  return upper_last_rx_timestamp_ms;
}
