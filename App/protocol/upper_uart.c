#include "upper_uart.h"

#include "chassis_config.h"
#include "chassis_control.h"
#include "chassis_layout.h"
#include "cmsis_os2.h"
#include "control_manager.h"
#include "encoder_driver.h"
#include "imu_bmi270.h"
#include "line_control.h"
#include "main.h"
#include "motor_driver.h"
#include "system_monitor.h"
#include "upper_protocol.h"
#include "usart.h"

#include <string.h>

#define UPPER_UART_RX_BUFFER_SIZE 128U
#define UPPER_UART_TX_QUEUE_CAPACITY 4U
#define UPPER_UART_TX_SLOT_INVALID 0xFFU

typedef enum
{
  UPPER_RX_WAIT_HEAD0 = 0,
  UPPER_RX_WAIT_HEAD1,
  UPPER_RX_WAIT_LEN,
  UPPER_RX_WAIT_BODY
} upper_rx_state_t;

typedef enum
{
  UPPER_TX_PRIORITY_IMU = 0,
  UPPER_TX_PRIORITY_STATUS = 1
} upper_tx_priority_t;

typedef struct
{
  uint8_t data[UPPER_PROTOCOL_MAX_FRAME];
  uint16_t length;
  uint32_t sequence;
  uint8_t priority;
  uint8_t used;
} upper_tx_slot_t;

static uint8_t upper_rx_dma_buffer[UPPER_UART_RX_BUFFER_SIZE] __attribute__((aligned(4)));
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
static uint8_t upper_parser_idle_cycles;
static uint16_t upper_last_write_pos;
static uint8_t upper_imu_tx_frame[UPPER_PROTOCOL_MAX_FRAME];
static uint8_t upper_imu_payload[UPPER_PROTOCOL_IMU_STATUS_PAYLOAD_LEN];
static uint32_t upper_last_imu_status_ms;
static upper_tx_slot_t upper_tx_queue[UPPER_UART_TX_QUEUE_CAPACITY];
static uint8_t upper_tx_queue_count;
static uint8_t upper_tx_active_slot;
static uint32_t upper_tx_sequence;

#define UPPER_PARSER_TIMEOUT_CYCLES  20U  /* 20 × 5ms = 100ms 无字节则重置解析器 */

static void UpperUart_ResetParser(void)
{
  upper_rx_state = UPPER_RX_WAIT_HEAD0;
  upper_frame_len = 0U;
  upper_frame_index = 0U;
}

static uint8_t UpperUart_SelectNextTxLocked(void)
{
  uint8_t selected = UPPER_UART_TX_SLOT_INVALID;

  for (uint8_t i = 0U; i < UPPER_UART_TX_QUEUE_CAPACITY; ++i)
  {
    if (upper_tx_queue[i].used == 0U || i == upper_tx_active_slot)
    {
      continue;
    }
    if (selected == UPPER_UART_TX_SLOT_INVALID ||
        upper_tx_queue[i].priority > upper_tx_queue[selected].priority ||
        (upper_tx_queue[i].priority == upper_tx_queue[selected].priority &&
         upper_tx_queue[i].sequence < upper_tx_queue[selected].sequence))
    {
      selected = i;
    }
  }
  return selected;
}

static uint8_t UpperUart_DropQueuedImuLocked(void)
{
  uint8_t selected = UPPER_UART_TX_SLOT_INVALID;

  for (uint8_t i = 0U; i < UPPER_UART_TX_QUEUE_CAPACITY; ++i)
  {
    if (upper_tx_queue[i].used == 0U ||
        i == upper_tx_active_slot ||
        upper_tx_queue[i].priority != UPPER_TX_PRIORITY_IMU)
    {
      continue;
    }
    if (selected == UPPER_UART_TX_SLOT_INVALID ||
        upper_tx_queue[i].sequence < upper_tx_queue[selected].sequence)
    {
      selected = i;
    }
  }
  if (selected == UPPER_UART_TX_SLOT_INVALID)
  {
    return 0U;
  }
  upper_tx_queue[selected].used = 0U;
  upper_tx_queue_count--;
  upper_state.tx_busy_drops++;
  return 1U;
}

static void UpperUart_TryStartTxLocked(void)
{
  uint8_t selected;
  HAL_StatusTypeDef status;

  if (upper_tx_active_slot != UPPER_UART_TX_SLOT_INVALID)
  {
    return;
  }
  selected = UpperUart_SelectNextTxLocked();
  if (selected == UPPER_UART_TX_SLOT_INVALID)
  {
    return;
  }
  status = HAL_UART_Transmit_IT(&huart3,
                                upper_tx_queue[selected].data,
                                upper_tx_queue[selected].length);
  if (status == HAL_OK)
  {
    upper_tx_active_slot = selected;
  }
  else if (status != HAL_BUSY)
  {
    upper_tx_queue[selected].used = 0U;
    upper_tx_queue_count--;
    upper_state.tx_busy_drops++;
  }
}

static uint8_t UpperUart_EnqueueTx(const uint8_t *frame,
                                   uint16_t frame_len,
                                   upper_tx_priority_t priority)
{
  uint32_t primask;
  uint8_t selected = UPPER_UART_TX_SLOT_INVALID;

  if (frame == 0 || frame_len == 0U || frame_len > UPPER_PROTOCOL_MAX_FRAME)
  {
    return 0U;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  if (upper_tx_queue_count >= UPPER_UART_TX_QUEUE_CAPACITY)
  {
    if (priority != UPPER_TX_PRIORITY_STATUS || UpperUart_DropQueuedImuLocked() == 0U)
    {
      upper_state.tx_busy_drops++;
      __set_PRIMASK(primask);
      return 0U;
    }
  }
  for (uint8_t i = 0U; i < UPPER_UART_TX_QUEUE_CAPACITY; ++i)
  {
    if (upper_tx_queue[i].used == 0U)
    {
      selected = i;
      break;
    }
  }
  if (selected == UPPER_UART_TX_SLOT_INVALID)
  {
    upper_state.tx_busy_drops++;
    __set_PRIMASK(primask);
    return 0U;
  }
  memcpy(upper_tx_queue[selected].data, frame, frame_len);
  upper_tx_queue[selected].length = frame_len;
  upper_tx_queue[selected].priority = (uint8_t)priority;
  upper_tx_queue[selected].sequence = upper_tx_sequence++;
  upper_tx_queue[selected].used = 1U;
  upper_tx_queue_count++;
  UpperUart_TryStartTxLocked();
  __set_PRIMASK(primask);
  return 1U;
}

static void UpperUart_ClearUartFlags(void)
{
  __HAL_UART_CLEAR_OREFLAG(&huart3);
  __HAL_UART_CLEAR_NEFLAG(&huart3);
  __HAL_UART_CLEAR_FEFLAG(&huart3);
  __HAL_UART_CLEAR_PEFLAG(&huart3);
}

static void UpperUart_RestartRxDma(uint8_t count_restart)
{
  (void)HAL_UART_DMAStop(&huart3);
  UpperUart_ClearUartFlags();
  upper_rx_read_pos = 0U;
  upper_last_write_pos = 0U;
  upper_parser_idle_cycles = 0U;
  UpperUart_ResetParser();
  (void)HAL_UART_Receive_DMA(&huart3, upper_rx_dma_buffer, UPPER_UART_RX_BUFFER_SIZE);
  if (count_restart != 0U)
  {
    upper_state.rx_resync_restarts++;
  }
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
      (void)velocity.mode; /* reserved: control mode byte */
      ControlManager_SetCommand(&chassis_cmd);
    }
  }
  else if (cmd == UPPER_CMD_ESTOP && payload_len == UPPER_PROTOCOL_ESTOP_PAYLOAD_LEN)
  {
    if (UpperProtocol_RemoteEstopSetRequested(payload, payload_len) != 0U)
    {
      ControlManager_SetEmergencyStop(1U);
    }
  }
  else if (cmd == UPPER_CMD_LINE_CTRL && payload_len == UPPER_PROTOCOL_LINE_CTRL_PAYLOAD_LEN)
  {
    LineControl_Enable((payload[0] != 0U) ? 1U : 0U);
  }
  else if (cmd == UPPER_CMD_CLEAR_FAULT && payload_len == UPPER_PROTOCOL_CLEAR_FAULT_PAYLOAD_LEN)
  {
    if (ControlManager_IsEmergencyStop() == 0U)
    {
      SystemMonitor_ClearLatchedFaults(0xFFFFFFFFUL);
    }
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
          upper_state.last_valid_frame_ms = upper_last_rx_timestamp_ms;
        }
        else
        {
          upper_state.rx_checksum_errors++;
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

  if (upper_rx_read_pos == write_pos)
  {
    /* 无新字节：若解析器处于中间状态，超时后重置 */
    if (upper_rx_state != UPPER_RX_WAIT_HEAD0)
    {
      upper_parser_idle_cycles++;
      if (upper_parser_idle_cycles >= UPPER_PARSER_TIMEOUT_CYCLES)
      {
        upper_state.rx_timeout_resets++;
        UpperUart_RestartRxDma(1U);
      }
    }
    return;
  }

  upper_parser_idle_cycles = 0U;
  {
    uint16_t unread = (write_pos >= upper_rx_read_pos) ?
                      (uint16_t)(write_pos - upper_rx_read_pos) :
                      (uint16_t)(UPPER_UART_RX_BUFFER_SIZE - upper_rx_read_pos + write_pos);
    uint16_t advanced = (write_pos >= upper_last_write_pos) ?
                        (uint16_t)(write_pos - upper_last_write_pos) :
                        (uint16_t)(UPPER_UART_RX_BUFFER_SIZE - upper_last_write_pos + write_pos);
    if (advanced > 0U && unread >= (UPPER_UART_RX_BUFFER_SIZE - UPPER_PROTOCOL_MAX_FRAME))
    {
      upper_state.rx_overwrite_count++;
      UpperUart_RestartRxDma(1U);
      return;
    }
    upper_last_write_pos = write_pos;
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
  motor_driver_state_t motor_state;
  system_monitor_state_t monitor_state;
  uint8_t payload_len;
  uint16_t frame_len;

  if ((now_ms - upper_last_status_ms) < UPPER_UART_STATUS_PERIOD_MS)
  {
    return;
  }

  ChassisControl_GetState(&chassis_state);
  EncoderDriver_GetState(&encoder_state);
  MotorDriver_GetState(&motor_state);
  SystemMonitor_GetState(&monitor_state);

  status.battery_voltage = monitor_state.battery_voltage;
  status.error_flags = monitor_state.error_flags;
  status.latched_error_flags = monitor_state.latched_error_flags;
  status.control_source = monitor_state.control_mode;
  if (ControlManager_IsEmergencyStop() != 0U)
  {
    status.status_flags |= UPPER_STATUS_FLAG_ESTOP;
  }
  if (ControlManager_IsFaultStop() != 0U)
  {
    status.status_flags |= UPPER_STATUS_FLAG_FAULT_STOP;
  }
  if (LineControl_IsEnabled() != 0U)
  {
    status.status_flags |= UPPER_STATUS_FLAG_LINE_ENABLED;
  }
  if (encoder_state.speed_valid_all != 0U)
  {
    status.status_flags |= UPPER_STATUS_FLAG_SPEED_VALID_ALL;
  }
  for (uint8_t i = 0U; i < UPPER_PROTOCOL_MOTOR_COUNT; ++i)
  {
    motor_id_t motor = (motor_id_t)i;
    status.motor_speed_mps[i] = chassis_state.motor_actual_mps[i];
    status.encoder_count[i] = encoder_state.count[i];
    status.motor_current_a[i] = monitor_state.motor_current_a[i];
    status.motor_target_mps[i] = chassis_state.motor_target_mps[i];
    status.motor_output_permille[i] = motor_state.effective_pwm[i];
    if (ChassisLayout_MotorEnabled(motor) != 0U)
    {
      status.motor_enabled_mask |= (uint8_t)(1U << i);
    }
    if (encoder_state.speed_valid[i] != 0U)
    {
      status.motor_speed_valid_mask |= (uint8_t)(1U << i);
    }
    if (encoder_state.anomaly_count[i] > 0U)
    {
      status.encoder_anomaly_mask |= (uint8_t)(1U << i);
    }
  }

  if (upper_state.rx_checksum_errors > 0U)  { status.comm_health_flags |= UPPER_COMM_HEALTH_CRC_ERR; }
  if (upper_state.rx_timeout_resets > 0U)   { status.comm_health_flags |= UPPER_COMM_HEALTH_TIMEOUT; }
  if (upper_state.tx_busy_drops > 0U)       { status.comm_health_flags |= UPPER_COMM_HEALTH_TX_DROP; }
  if (upper_state.rx_overwrite_count > 0U)  { status.comm_health_flags |= UPPER_COMM_HEALTH_TIMEOUT; }

  payload_len = UpperProtocol_BuildStatusPayload(&status, upper_status_payload, sizeof(upper_status_payload));
  frame_len = UpperProtocol_BuildFrame(UPPER_CMD_STATUS, upper_status_payload, payload_len, upper_tx_frame, sizeof(upper_tx_frame));
  if (frame_len > 0U)
  {
    if (UpperUart_EnqueueTx(upper_tx_frame, frame_len, UPPER_TX_PRIORITY_STATUS) != 0U)
    {
      upper_last_status_ms = now_ms;
    }
  }
}

void UpperUart_Init(void)
{
  upper_rx_read_pos = 0U;
  upper_last_write_pos = 0U;
  upper_last_status_ms = 0U;
  upper_last_imu_status_ms = 0U;
  upper_state = (upper_uart_state_t){0};
  memset(upper_tx_queue, 0, sizeof(upper_tx_queue));
  upper_tx_queue_count = 0U;
  upper_tx_active_slot = UPPER_UART_TX_SLOT_INVALID;
  upper_tx_sequence = 0UL;
  upper_last_rx_timestamp_ms = 0U;
  UpperUart_ResetParser();
  (void)HAL_UART_Receive_DMA(&huart3, upper_rx_dma_buffer, UPPER_UART_RX_BUFFER_SIZE);
}

static void UpperUart_SendImuStatus(uint32_t now_ms)
{
  imu_bmi270_state_t imu_state;
  upper_imu_status_payload_t imu;
  uint8_t payload_len;
  uint16_t frame_len;

  if ((now_ms - upper_last_imu_status_ms) < UPPER_IMU_STATUS_PERIOD_MS)
  {
    return;
  }

  ImuBmi270_GetState(&imu_state);
  if (imu_state.online == 0U)
  {
    return;
  }

  imu = (upper_imu_status_payload_t){0};
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    imu.accel_g[i] = imu_state.body_accel_g[i];
    imu.gyro_corrected_dps[i] = imu_state.body_gyro_dps[i];
  }
  imu.euler_deg[0] = imu_state.roll_deg;
  imu.euler_deg[1] = imu_state.pitch_deg;
  imu.euler_deg[2] = imu_state.yaw_deg;
  for (uint8_t i = 0U; i < 4U; ++i)
  {
    imu.quaternion[i] = imu_state.quaternion[i];
  }
  imu.timestamp_ms = now_ms;
  imu.sensor_time = imu_state.sensor_time;
  imu.sample_count = imu_state.sample_count;
  imu.quality_flags = imu_state.quality_flags;
  imu.quality_counters[0] = imu_state.spi_error_count;
  imu.quality_counters[1] = imu_state.init_failure_count;
  imu.quality_counters[2] = imu_state.fifo_overflow_count;
  imu.quality_counters[3] = imu_state.timestamp_error_count;
  imu.quality_counters[4] = imu_state.gyro_saturation_count;
  imu.quality_counters[5] = imu_state.accel_anomaly_count;
  imu.quality_counters[6] = imu_state.attitude_invalid_count;
  if (imu_state.online != 0U)       { imu.status_flags |= UPPER_IMU_FLAG_ONLINE; }
  if (imu_state.gyro_calibrated != 0U) { imu.status_flags |= UPPER_IMU_FLAG_CALIBRATED; }
  if (imu_state.quality_flags != 0UL || imu_state.last_error != IMU_BMI270_ERROR_NONE) { imu.status_flags |= UPPER_IMU_FLAG_ERROR; }
  if (imu_state.sensor_time_valid != 0U) { imu.status_flags |= UPPER_IMU_FLAG_SENSOR_TIME; }
  imu.temperature_c = (int8_t)((imu_state.temperature_c >= 0.0f) ?
                               (imu_state.temperature_c + 0.5f) :
                               (imu_state.temperature_c - 0.5f));

  payload_len = UpperProtocol_BuildImuStatusPayload(&imu, upper_imu_payload, sizeof(upper_imu_payload));
  frame_len = UpperProtocol_BuildFrame(UPPER_CMD_IMU_STATUS, upper_imu_payload, payload_len, upper_imu_tx_frame, sizeof(upper_imu_tx_frame));
  if (frame_len > 0U)
  {
    if (UpperUart_EnqueueTx(upper_imu_tx_frame, frame_len, UPPER_TX_PRIORITY_IMU) != 0U)
    {
      upper_last_imu_status_ms = now_ms;
    }
  }
}

void UpperUart_Update(void)
{
  uint32_t now_ms = osKernelGetTickCount();

  UpperUart_PollRx();
  UpperUart_SendStatus(now_ms);
  UpperUart_SendImuStatus(now_ms);
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

void UpperUart_OnUartError(void)
{
  uint32_t primask;

  upper_state.uart_errors++;
  UpperUart_RestartRxDma(1U);
  primask = __get_PRIMASK();
  __disable_irq();
  if (upper_tx_active_slot != UPPER_UART_TX_SLOT_INVALID &&
      upper_tx_queue[upper_tx_active_slot].used != 0U)
  {
    upper_tx_queue[upper_tx_active_slot].used = 0U;
    upper_tx_queue_count--;
    upper_state.tx_busy_drops++;
  }
  upper_tx_active_slot = UPPER_UART_TX_SLOT_INVALID;
  UpperUart_TryStartTxLocked();
  __set_PRIMASK(primask);
}

uint32_t UpperUart_GetLastRxTimestamp(void)
{
  return upper_last_rx_timestamp_ms;
}

void UpperUart_OnDmaHalf(void)
{
  upper_state.rx_dma_half_count++;
}

void UpperUart_OnDmaFull(void)
{
  upper_state.rx_dma_full_count++;
}

void UpperUart_OnTxComplete(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (upper_tx_active_slot != UPPER_UART_TX_SLOT_INVALID &&
      upper_tx_queue[upper_tx_active_slot].used != 0U)
  {
    upper_tx_queue[upper_tx_active_slot].used = 0U;
    upper_tx_queue_count--;
    upper_state.tx_frames++;
  }
  upper_tx_active_slot = UPPER_UART_TX_SLOT_INVALID;
  UpperUart_TryStartTxLocked();
  __set_PRIMASK(primask);
}
