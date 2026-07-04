#include "esp12f_comm.h"

#include "chassis_config.h"
#include "chassis_control.h"
#include "chassis_layout.h"
#include "cmsis_os2.h"
#include "control_manager.h"
#include "encoder_driver.h"
#include "esp12f_flash_bridge.h"
#include "line_control.h"
#include "system_monitor.h"
#include "upper_protocol.h"
#include "usart.h"

#define ESP12F_RX_RING_SIZE 128U

typedef enum
{
  ESP12F_RX_WAIT_HEAD0 = 0,
  ESP12F_RX_WAIT_HEAD1,
  ESP12F_RX_WAIT_LEN,
  ESP12F_RX_WAIT_BODY
} esp12f_rx_state_t;

static volatile uint8_t esp12f_rx_ring[ESP12F_RX_RING_SIZE] __attribute__((aligned(4)));
static volatile uint16_t esp12f_rx_head;
static volatile uint16_t esp12f_rx_tail;
static uint8_t esp12f_rx_byte;
static esp12f_rx_state_t esp12f_rx_state;
static uint8_t esp12f_frame_buf[UPPER_PROTOCOL_MAX_PAYLOAD + 3U];
static uint8_t esp12f_frame_len;
static uint8_t esp12f_frame_index;
static uint8_t esp12f_tx_frame[UPPER_PROTOCOL_MAX_FRAME];
static uint8_t esp12f_status_payload[UPPER_PROTOCOL_STATUS_PAYLOAD_LEN];
static uint32_t esp12f_last_status_ms;
static esp12f_comm_state_t esp12f_state;
static uint8_t esp12f_isolated;

static void Esp12fComm_ResetParser(void)
{
  esp12f_rx_state = ESP12F_RX_WAIT_HEAD0;
  esp12f_frame_len = 0U;
  esp12f_frame_index = 0U;
}

static void Esp12fComm_HandleFrame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
  esp12f_state.rx_frames++;
  /* Record RX timestamp for OLED module online detection */
  esp12f_state.last_rx_timestamp_ms = osKernelGetTickCount();
  if (cmd == UPPER_CMD_SET_VELOCITY)
  {
    upper_velocity_payload_t velocity;
    if (UpperProtocol_ParseVelocityPayload(payload, payload_len, &velocity) != 0U)
    {
      chassis_cmd_t chassis_cmd = {
        .linear_x = velocity.linear_x,
        .angular_z = velocity.angular_z,
        .enable = velocity.enable,
        .source = CONTROL_SOURCE_ESP12F,
        .timestamp_ms = osKernelGetTickCount(),
      };
      (void)velocity.mode; /* reserved: control mode byte */
      (void)ControlManager_SetCommand(&chassis_cmd);
    }
  }
  else if (cmd == UPPER_CMD_ESTOP && payload_len == UPPER_PROTOCOL_ESTOP_PAYLOAD_LEN)
  {
    ControlManager_SetEmergencyStop(payload[0]);
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

static void Esp12fComm_ProcessByte(uint8_t byte)
{
  switch (esp12f_rx_state)
  {
    case ESP12F_RX_WAIT_HEAD0:
      if (byte == UPPER_PROTOCOL_HEAD_0)
      {
        esp12f_rx_state = ESP12F_RX_WAIT_HEAD1;
      }
      break;
    case ESP12F_RX_WAIT_HEAD1:
      esp12f_rx_state = (byte == UPPER_PROTOCOL_HEAD_1) ? ESP12F_RX_WAIT_LEN : ESP12F_RX_WAIT_HEAD0;
      break;
    case ESP12F_RX_WAIT_LEN:
      if (byte == 0U || byte > UPPER_PROTOCOL_CMD_LEN(UPPER_PROTOCOL_MAX_PAYLOAD))
      {
        esp12f_state.rx_length_errors++;
        Esp12fComm_ResetParser();
      }
      else
      {
        esp12f_frame_buf[0] = byte;
        esp12f_frame_len = byte;
        esp12f_frame_index = 0U;
        esp12f_rx_state = ESP12F_RX_WAIT_BODY;
      }
      break;
    case ESP12F_RX_WAIT_BODY:
      esp12f_frame_index++;
      esp12f_frame_buf[esp12f_frame_index] = byte;
      if (esp12f_frame_index >= (uint8_t)(esp12f_frame_len + 1U))
      {
        uint8_t checksum = esp12f_frame_buf[esp12f_frame_index];
        uint8_t expect = UpperProtocol_Checksum8(esp12f_frame_buf, (uint16_t)esp12f_frame_len + 1U);
        if (checksum == expect)
        {
          uint8_t frame_cmd = esp12f_frame_buf[1];
          const uint8_t *frame_payload = &esp12f_frame_buf[2];
          uint8_t frame_payload_len = (uint8_t)(esp12f_frame_len - 1U);
          Esp12fComm_HandleFrame(frame_cmd, frame_payload, frame_payload_len);
        }
        else
        {
          esp12f_state.rx_checksum_errors++;
        }
        Esp12fComm_ResetParser();
      }
      break;
    default:
      Esp12fComm_ResetParser();
      break;
  }
}

static void Esp12fComm_PollRx(void)
{
  while (esp12f_rx_tail != esp12f_rx_head)
  {
    uint8_t byte = esp12f_rx_ring[esp12f_rx_tail];
    esp12f_rx_tail = (uint16_t)((esp12f_rx_tail + 1U) % ESP12F_RX_RING_SIZE);
    Esp12fComm_ProcessByte(byte);
  }
}

static void Esp12fComm_SendStatus(uint32_t now_ms)
{
  upper_status_payload_t status = {0};
  chassis_control_state_t chassis_state;
  encoder_state_t encoder_state;
  system_monitor_state_t monitor_state;
  uint8_t payload_len;
  uint16_t frame_len;

  if ((now_ms - esp12f_last_status_ms) < ESP12F_STATUS_PERIOD_MS)
  {
    return;
  }
  esp12f_last_status_ms = now_ms;
  if (huart2.gState != HAL_UART_STATE_READY)
  {
    esp12f_state.tx_busy_drops++;
    return;
  }

  ChassisControl_GetState(&chassis_state);
  EncoderDriver_GetState(&encoder_state);
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
    status.motor_output_permille[i] = chassis_state.motor_output_permille[i];
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

  if (esp12f_state.rx_checksum_errors > 0U)   { status.comm_health_flags |= UPPER_COMM_HEALTH_ESP_CRC; }
  if (esp12f_state.rx_overflow_errors > 0U)    { status.comm_health_flags |= UPPER_COMM_HEALTH_ESP_TIMEOUT; }
  if (esp12f_state.tx_busy_drops > 0U)         { status.comm_health_flags |= UPPER_COMM_HEALTH_ESP_TX_DROP; }

  payload_len = UpperProtocol_BuildStatusPayload(&status, esp12f_status_payload, sizeof(esp12f_status_payload));
  frame_len = UpperProtocol_BuildFrame(UPPER_CMD_STATUS, esp12f_status_payload, payload_len, esp12f_tx_frame, sizeof(esp12f_tx_frame));
  if (frame_len > 0U)
  {
    if (HAL_UART_Transmit_IT(&huart2, esp12f_tx_frame, frame_len) == HAL_OK)
    {
      esp12f_state.tx_frames++;
    }
    else
    {
      esp12f_state.tx_busy_drops++;
    }
  }
}

void Esp12fComm_Init(void)
{
  esp12f_isolated = 0U;
  esp12f_rx_head = 0U;
  esp12f_rx_tail = 0U;
  esp12f_last_status_ms = 0U;
  esp12f_state = (esp12f_comm_state_t){0};
  esp12f_state.last_rx_timestamp_ms = 0U;
  Esp12fComm_SetDownloadMode(0U);
  Esp12fComm_RestartRx();
}

void Esp12fComm_RestartRx(void)
{
  esp12f_rx_head = 0U;
  esp12f_rx_tail = 0U;
  Esp12fComm_ResetParser();
  (void)HAL_UART_Receive_IT(&huart2, &esp12f_rx_byte, 1U);
}

void Esp12fComm_Update(void)
{
  uint32_t now_ms = osKernelGetTickCount();

  if (esp12f_isolated != 0U)
  {
    return;
  }

  if (Esp12fFlashBridge_IsActive() != 0U)
  {
    return;
  }

  Esp12fComm_PollRx();
  Esp12fComm_SendStatus(now_ms);
}

void Esp12fComm_ResetModule(void)
{
  HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(5U);
  HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_SET);
}

void Esp12fComm_Isolate(void)
{
  esp12f_isolated = 1U;
  HAL_NVIC_DisableIRQ(USART2_IRQn);
  CLEAR_BIT(huart2.Instance->CR3, USART_CR3_DMAR);
  huart2.hdmarx = 0;
  (void)HAL_UART_Abort(&huart2);
  HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
  HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_RESET);
}

void Esp12fComm_SetDownloadMode(uint8_t enabled)
{
  esp12f_state.boot_mode_download = (enabled != 0U) ? 1U : 0U;
  HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ESP_IO0_GPIO_Port, ESP_IO0_Pin, (enabled != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_SET);
}

void Esp12fComm_OnRxCplt(void)
{
  uint16_t next_head = (uint16_t)((esp12f_rx_head + 1U) % ESP12F_RX_RING_SIZE);

  if (next_head != esp12f_rx_tail)
  {
    esp12f_rx_ring[esp12f_rx_head] = esp12f_rx_byte;
    esp12f_rx_head = next_head;
  }
  else
  {
    esp12f_state.rx_overflow_errors++;
  }
  (void)HAL_UART_Receive_IT(&huart2, &esp12f_rx_byte, 1U);
}

void Esp12fComm_OnUartError(void)
{
  (void)HAL_UART_Receive_IT(&huart2, &esp12f_rx_byte, 1U);
}

void Esp12fComm_GetState(esp12f_comm_state_t *state)
{
  if (state != 0)
  {
    *state = esp12f_state;
  }
}
