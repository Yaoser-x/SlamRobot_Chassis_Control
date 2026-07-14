#include "esp12f_service.h"
#include "platform_time.h"
#include "esp12f_transport.h"

#include "control_config.h"

#include "adc_monitor.h"

#include "chassis_service.h"

#include "chassis_layout.h"

#include "control_service.h"

#include "encoder_driver.h"

#include "line_control_service.h"

#include "imu_bmi270.h"

#include "motor_driver.h"

#include "post_service.h"

#include "reset_reason_service.h"

#include "safety_service.h"

#include "upper_protocol.h"

#define ESP12F_RX_RING_SIZE            128U
#define ESP12F_RX_INTERBYTE_TIMEOUT_MS 100U

typedef enum
{
    ESP12F_RX_WAIT_HEAD0 = 0,
    ESP12F_RX_WAIT_HEAD1,
    ESP12F_RX_WAIT_LEN,
    ESP12F_RX_WAIT_BODY
} esp12f_rx_state_t;

static volatile uint8_t       esp12f_rx_ring[ESP12F_RX_RING_SIZE] __attribute__((aligned(4)));
static volatile uint16_t      esp12f_rx_head;
static volatile uint16_t      esp12f_rx_tail;
static uint8_t                esp12f_rx_byte;
static esp12f_rx_state_t      esp12f_rx_state;
static uint8_t                esp12f_frame_buf[UPPER_PROTOCOL_MAX_PAYLOAD + 3U];
static uint8_t                esp12f_frame_len;
static uint8_t                esp12f_frame_index;
static uint32_t               esp12f_parser_last_byte_ms;
static uint8_t                esp12f_tx_frame[UPPER_PROTOCOL_MAX_FRAME];
static uint8_t                esp12f_status_payload[UPPER_PROTOCOL_STATUS_PAYLOAD_LEN];
static uint32_t               esp12f_last_status_ms;
static uint8_t                esp12f_diagnostic_tx_frame[UPPER_PROTOCOL_MAX_FRAME];
static uint8_t                esp12f_diagnostic_payload[UPPER_PROTOCOL_DIAGNOSTIC_PAYLOAD_LEN];
static uint32_t               esp12f_last_diagnostic_ms;
static esp12f_service_state_t esp12f_state;
static uint8_t                esp12f_isolated;

static void Esp12fService_ResetParser(void)
{
    esp12f_rx_state            = ESP12F_RX_WAIT_HEAD0;
    esp12f_frame_len           = 0U;
    esp12f_frame_index         = 0U;
    esp12f_parser_last_byte_ms = 0U;
}

static void Esp12fService_HandleFrame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
    esp12f_state.rx_frames++;
    /* Record RX timestamp for OLED module online detection */
    esp12f_state.last_rx_timestamp_ms = PlatformTime_TaskNowMs();
    if (cmd == UPPER_CMD_SET_VELOCITY)
    {
        upper_velocity_payload_t velocity;
        if (UpperProtocol_ParseVelocityPayload(payload, payload_len, &velocity) != 0U)
        {
            chassis_cmd_t chassis_cmd = {
                .linear_x     = velocity.linear_x,
                .angular_z    = velocity.angular_z,
                .enable       = velocity.enable,
                .source       = CONTROL_SOURCE_ESP12F,
                .timestamp_ms = PlatformTime_TaskNowMs(),
            };
            (void)velocity.mode; /* reserved: control mode byte */
            (void)ControlService_SetCommand(&chassis_cmd);
        }
    }
    else if (cmd == UPPER_CMD_ESTOP && payload_len == UPPER_PROTOCOL_ESTOP_PAYLOAD_LEN)
    {
        if (UpperProtocol_RemoteEstopSetRequested(payload, payload_len) != 0U)
        {
            ControlService_SetEmergencyStop(1U);
        }
    }
    else if (cmd == UPPER_CMD_LINE_CTRL && payload_len == UPPER_PROTOCOL_LINE_CTRL_PAYLOAD_LEN)
    {
        LineControlService_Enable((payload[0] != 0U) ? 1U : 0U);
    }
    else if (cmd == UPPER_CMD_CLEAR_FAULT && payload_len == UPPER_PROTOCOL_CLEAR_FAULT_PAYLOAD_LEN)
    {
        if (ControlService_IsEmergencyStop() == 0U)
        {
            SafetyService_ClearLatchedFaults(0xFFFFFFFFUL);
        }
    }
}

static void Esp12fService_ProcessByte(uint8_t byte)
{
    esp12f_parser_last_byte_ms = PlatformTime_TaskNowMs();
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
                Esp12fService_ResetParser();
            }
            else
            {
                esp12f_frame_buf[0] = byte;
                esp12f_frame_len    = byte;
                esp12f_frame_index  = 0U;
                esp12f_rx_state     = ESP12F_RX_WAIT_BODY;
            }
            break;
        case ESP12F_RX_WAIT_BODY:
            esp12f_frame_index++;
            esp12f_frame_buf[esp12f_frame_index] = byte;
            if (esp12f_frame_index >= (uint8_t)(esp12f_frame_len + 1U))
            {
                uint8_t checksum = esp12f_frame_buf[esp12f_frame_index];
                uint8_t expect   = UpperProtocol_Checksum8(esp12f_frame_buf, (uint16_t)esp12f_frame_len + 1U);
                if (checksum == expect)
                {
                    uint8_t        frame_cmd         = esp12f_frame_buf[1];
                    const uint8_t *frame_payload     = &esp12f_frame_buf[2];
                    uint8_t        frame_payload_len = (uint8_t)(esp12f_frame_len - 1U);
                    Esp12fService_HandleFrame(frame_cmd, frame_payload, frame_payload_len);
                }
                else
                {
                    esp12f_state.rx_checksum_errors++;
                }
                Esp12fService_ResetParser();
            }
            break;
        default:
            Esp12fService_ResetParser();
            break;
    }
}

static void Esp12fService_PollRx(void)
{
    while (esp12f_rx_tail != esp12f_rx_head)
    {
        uint8_t byte   = esp12f_rx_ring[esp12f_rx_tail];
        esp12f_rx_tail = (uint16_t)((esp12f_rx_tail + 1U) % ESP12F_RX_RING_SIZE);
        Esp12fService_ProcessByte(byte);
    }
}

static void Esp12fService_SendStatus(uint32_t now_ms)
{
    upper_status_payload_t     status = {0};
    chassis_service_snapshot_t chassis_state;
    encoder_state_t            encoder_state;
    motor_driver_state_t       motor_state;
    safety_service_snapshot_t  monitor_state;
    uint8_t                    payload_len;
    uint16_t                   frame_len;

    if ((now_ms - esp12f_last_status_ms) < ESP12F_STATUS_PERIOD_MS)
    {
        return;
    }
    esp12f_last_status_ms = now_ms;
    if (Esp12fTransport_IsTxReady() == 0U)
    {
        esp12f_state.tx_busy_drops++;
        return;
    }

    ChassisService_GetState(&chassis_state);
    EncoderDriver_GetState(&encoder_state);
    MotorDriver_GetState(&motor_state);
    SafetyService_GetState(&monitor_state);

    status.battery_voltage     = monitor_state.battery_voltage;
    status.error_flags         = monitor_state.error_flags;
    status.latched_error_flags = monitor_state.latched_error_flags;
    status.control_source      = monitor_state.control_mode;
    if (ControlService_IsEmergencyStop() != 0U)
    {
        status.status_flags |= UPPER_STATUS_FLAG_ESTOP;
    }
    if (ControlService_IsFaultStop() != 0U)
    {
        status.status_flags |= UPPER_STATUS_FLAG_FAULT_STOP;
    }
    if (LineControlService_IsEnabled() != 0U)
    {
        status.status_flags |= UPPER_STATUS_FLAG_LINE_ENABLED;
    }
    if (encoder_state.speed_valid_all != 0U)
    {
        status.status_flags |= UPPER_STATUS_FLAG_SPEED_VALID_ALL;
    }
    for (uint8_t i = 0U; i < UPPER_PROTOCOL_MOTOR_COUNT; ++i)
    {
        motor_id_t motor                = (motor_id_t)i;
        status.motor_speed_mps[i]       = chassis_state.motor_actual_mps[i];
        status.encoder_count[i]         = encoder_state.count[i];
        status.motor_current_a[i]       = monitor_state.motor_current_a[i];
        status.motor_target_mps[i]      = chassis_state.motor_target_mps[i];
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

    if (esp12f_state.rx_checksum_errors > 0U)
    {
        status.comm_health_flags |= UPPER_COMM_HEALTH_ESP_CRC;
    }
    if (esp12f_state.rx_overflow_errors > 0U)
    {
        status.comm_health_flags |= UPPER_COMM_HEALTH_ESP_TIMEOUT;
    }
    if (esp12f_state.tx_busy_drops > 0U)
    {
        status.comm_health_flags |= UPPER_COMM_HEALTH_ESP_TX_DROP;
    }

    payload_len = UpperProtocol_BuildStatusPayload(&status, esp12f_status_payload, sizeof(esp12f_status_payload));
    frame_len   = UpperProtocol_BuildFrame(UPPER_CMD_STATUS,
                                         esp12f_status_payload,
                                         payload_len,
                                         esp12f_tx_frame,
                                         sizeof(esp12f_tx_frame));
    if (frame_len > 0U)
    {
        if (Esp12fTransport_TransmitAsync(esp12f_tx_frame, frame_len) == TRANSPORT_STATUS_OK)
        {
            esp12f_state.tx_frames++;
        }
        else
        {
            esp12f_state.tx_busy_drops++;
        }
    }
}

static void Esp12fService_SendDiagnostic(uint32_t now_ms)
{
    upper_diagnostic_payload_t diagnostic = {0};
    post_result_t              post;
    adc_monitor_state_t        adc;
    safety_service_snapshot_t  monitor;
    imu_bmi270_state_t         imu;
    uint8_t                    payload_len;
    uint16_t                   frame_len;

    if ((now_ms - esp12f_last_diagnostic_ms) < UPPER_DIAGNOSTIC_PERIOD_MS || Esp12fTransport_IsTxReady() == 0U)
    {
        return;
    }

    POST_GetResult(&post);
    AdcMonitor_GetState(&adc);
    SafetyService_GetState(&monitor);
    ImuBmi270_GetState(&imu);
    diagnostic.post_done = post.done;
    if (imu.online != 0U)
    {
        diagnostic.imu_status_flags |= UPPER_IMU_FLAG_ONLINE;
    }
    if (imu.gyro_calibrated != 0U)
    {
        diagnostic.imu_status_flags |= UPPER_IMU_FLAG_CALIBRATED;
    }
    if (imu.quality_flags != 0UL || imu.last_error != IMU_BMI270_ERROR_NONE)
    {
        diagnostic.imu_status_flags |= UPPER_IMU_FLAG_ERROR;
    }
    if (imu.sensor_time_valid != 0U)
    {
        diagnostic.imu_status_flags |= UPPER_IMU_FLAG_SENSOR_TIME;
    }
    diagnostic.post_error_flags         = post.error_flags;
    diagnostic.adc_invalid_reason_flags = adc.invalid_reason_flags;
    diagnostic.task_timeout_mask        = monitor.task_timeout_mask;
    diagnostic.imu_quality_flags        = imu.quality_flags;
    diagnostic.reset_reason_flags       = ResetReasonService_GetFlags();
    diagnostic.uptime_ms                = now_ms;
    payload_len =
        UpperProtocol_BuildDiagnosticPayload(&diagnostic, esp12f_diagnostic_payload, sizeof(esp12f_diagnostic_payload));
    frame_len = UpperProtocol_BuildFrame(UPPER_CMD_DIAGNOSTIC,
                                         esp12f_diagnostic_payload,
                                         payload_len,
                                         esp12f_diagnostic_tx_frame,
                                         sizeof(esp12f_diagnostic_tx_frame));
    if (frame_len > 0U && Esp12fTransport_TransmitAsync(esp12f_diagnostic_tx_frame, frame_len) == TRANSPORT_STATUS_OK)
    {
        esp12f_last_diagnostic_ms = now_ms;
        esp12f_state.tx_frames++;
    }
}

void Esp12fService_Init(void)
{
    esp12f_isolated                   = 0U;
    esp12f_rx_head                    = 0U;
    esp12f_rx_tail                    = 0U;
    esp12f_last_status_ms             = 0U;
    esp12f_last_diagnostic_ms         = 0U;
    esp12f_state                      = (esp12f_service_state_t){0};
    esp12f_state.last_rx_timestamp_ms = 0U;
    Esp12fService_SetDownloadMode(0U);
    Esp12fService_RestartRx();
}

void Esp12fService_RestartRx(void)
{
    esp12f_rx_head = 0U;
    esp12f_rx_tail = 0U;
    Esp12fService_ResetParser();
    Esp12fTransport_StartRx(&esp12f_rx_byte);
}

void Esp12fService_Update(void)
{
    uint32_t now_ms = PlatformTime_TaskNowMs();

    if (esp12f_isolated != 0U)
    {
        return;
    }

    if (esp12f_rx_state != ESP12F_RX_WAIT_HEAD0
        && (uint32_t)(now_ms - esp12f_parser_last_byte_ms) > ESP12F_RX_INTERBYTE_TIMEOUT_MS)
    {
        esp12f_state.rx_timeout_resets++;
        Esp12fService_ResetParser();
    }
    Esp12fService_PollRx();
    Esp12fService_SendStatus(now_ms);
    Esp12fService_SendDiagnostic(now_ms);
}

void Esp12fService_ResetModule(void)
{
    Esp12fTransport_ResetModule();
}

void Esp12fService_Isolate(void)
{
    esp12f_isolated = 1U;
    Esp12fTransport_Isolate();
}

void Esp12fService_SetDownloadMode(uint8_t enabled)
{
    esp12f_state.boot_mode_download = (enabled != 0U) ? 1U : 0U;
    Esp12fTransport_SetDownloadMode(enabled);
}

void Esp12fService_OnRxCplt(void)
{
    uint16_t next_head = (uint16_t)((esp12f_rx_head + 1U) % ESP12F_RX_RING_SIZE);

    if (next_head != esp12f_rx_tail)
    {
        esp12f_rx_ring[esp12f_rx_head] = esp12f_rx_byte;
        esp12f_rx_head                 = next_head;
    }
    else
    {
        esp12f_state.rx_overflow_errors++;
    }
    Esp12fTransport_StartRx(&esp12f_rx_byte);
}

void Esp12fService_OnUartError(void)
{
    esp12f_state.uart_errors++;
    esp12f_rx_head = 0U;
    esp12f_rx_tail = 0U;
    Esp12fService_ResetParser();
    Esp12fTransport_StartRx(&esp12f_rx_byte);
}

void Esp12fService_GetState(esp12f_service_state_t *state)
{
    if (state != 0)
    {
        *state = esp12f_state;
    }
}
