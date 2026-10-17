#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "chassis_service.h"
#include "power_adc_driver.h"
#include "motor_hardware_layout.h"
#include "control_service.h"
#include "wheel_encoder_driver.h"
#include "bmi270_driver.h"
#include "line_following_service.h"
#include "motor_driver.h"
#include "power_on_self_test_service.h"
#include "safety_management_service.h"
#include "communication_publish_model_types.h"
#include "state_estimation_status.h"
#include "host_communication_service.h"
#include "robot_link_protocol.h"
#include "usart.h"

static USART_TypeDef      usart3_instance  = {0};
static DMA_Stream_TypeDef usart3_rx_stream = {0};
static DMA_HandleTypeDef  hdma_usart3_rx   = {.Instance = &usart3_rx_stream};
UART_HandleTypeDef        huart3           = {
                     .Instance = &usart3_instance,
                     .Init     = {.BaudRate = 115200U},
                     .hdmarx   = &hdma_usart3_rx,
                     .gState   = HAL_UART_STATE_READY,
};

static uint8_t                      *rx_buffer;
static uint16_t                      rx_size;
static uint32_t                      rx_start_count;
static uint32_t                      dma_stop_count;
static uint32_t                      tx_dma_count;
static uint32_t                      tx_it_count;
static uint8_t                       tx_last_frame[ROBOT_LINK_PROTOCOL_MAX_FRAME];
static uint16_t                      tx_last_size;
static UART_HandleTypeDef           *tx_last_uart;
static uint32_t                      fake_tick;
static state_estimation_imu_status_t fake_imu_state;
static power_adc_driver_state_t      fake_adc_state;
static power_on_self_test_result_t   fake_post_result;
static uint32_t                      fake_primask;
static uint32_t                      estop_set_count;
static uint8_t                       estop_last_value;
static communication_publish_model_t fake_publish_model;
static uint8_t                       fake_encoder_anomaly_mask;
static uint32_t                      fake_encoder_anomaly_generation;
static uint8_t                       overwrite_dma_on_release;
static const communication_config_t  fake_communication_config = {
     .host_status_period_ms     = 50U,
     .host_imu_status_period_ms = 20U,
     .host_diagnostic_period_ms = 200U,
     .esp12f_status_period_ms   = 100U,
};
static const communication_firmware_identity_t fake_firmware_identity = {
    .hardware_revision = 0x00020000UL,
    .capabilities      = COMMUNICATION_REQUIRED_CAPABILITIES,
};

static void RefreshFakePublishModel(void);
#define HostCommunication_Init()   HostCommunication_Init(&fake_communication_config, &fake_firmware_identity)
#define HostCommunication_Update() (RefreshFakePublishModel(), HostCommunication_Update(&fake_publish_model))

uint32_t __get_PRIMASK(void)
{
    return fake_primask;
}

void __disable_irq(void)
{
    fake_primask = 1U;
}

void __set_PRIMASK(uint32_t primask)
{
    fake_primask = primask;
}

uint32_t osKernelGetTickCount(void)
{
    return fake_tick;
}

int32_t osDelayUntil(uint32_t ticks)
{
    fake_tick = ticks;
    return 0;
}

HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    if (huart == &huart3)
    {
        rx_buffer                     = pData;
        rx_size                       = Size;
        huart->hdmarx                 = &hdma_usart3_rx;
        huart->hdmarx->Instance->NDTR = Size;
        huart->Instance->CR3 |= USART_CR3_DMAR;
        rx_start_count++;
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
    tx_dma_count++;
    tx_last_uart = huart;
    if (pData != 0 && Size <= (uint16_t)sizeof(tx_last_frame))
    {
        for (uint16_t i = 0U; i < Size; ++i)
        {
            tx_last_frame[i] = pData[i];
        }
        tx_last_size = Size;
    }
    return HAL_BUSY;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
    if (huart->gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }
    tx_it_count++;
    tx_last_uart = huart;
    if (pData != 0 && Size <= (uint16_t)sizeof(tx_last_frame))
    {
        for (uint16_t i = 0U; i < Size; ++i)
        {
            tx_last_frame[i] = pData[i];
        }
        tx_last_size = Size;
    }
    huart->gState = HAL_UART_STATE_BUSY_TX;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        huart->Instance->CR3 &= ~(USART_CR3_DMAR | USART_CR3_DMAT);
        dma_stop_count++;
    }
    return HAL_OK;
}

control_command_result_t ControlService_SetCommand(const chassis_cmd_t *cmd)
{
    (void)cmd;
    return CONTROL_COMMAND_ACCEPTED;
}

command_result_t CommandManagement_Set(const command_velocity_t *command)
{
    chassis_cmd_t legacy = {
        .linear_x     = command->linear_x,
        .angular_z    = command->angular_z,
        .enable       = command->enable,
        .source       = (uint8_t)command->source,
        .timestamp_ms = command->timestamp_ms,
    };
    return (command_result_t)ControlService_SetCommand(&legacy);
}

void CommandManagement_ClearSource(command_source_t source)
{
    (void)source;
}

command_result_t CommandManagement_DisableRemoteSource(command_source_t source)
{
    CommandManagement_ClearSource(source);
    if (overwrite_dma_on_release != 0U)
    {
        overwrite_dma_on_release = 0U;
        for (uint16_t index = 0U; index < rx_size; ++index)
        {
            rx_buffer[index] = 0xFFU;
        }
    }
    return COMMAND_RESULT_ACCEPTED;
}

uint8_t CommandManagement_GetRefreshToken(command_source_t         source,
                                          uint32_t                 expected_slot_generation,
                                          command_refresh_token_t *token)
{
    *token = (command_refresh_token_t){
        .source              = source,
        .slot_generation     = expected_slot_generation,
        .accepted_command_id = 1UL,
        .revoke_generation   = 1UL,
        .mode                = 2U,
    };
    return 1U;
}

uint8_t CommandManagement_RefreshAccepted(const command_refresh_token_t *token, uint32_t now_ms)
{
    (void)token;
    (void)now_ms;
    return 1U;
}

uint32_t CommandManagement_GetMotionRevokeGeneration(void)
{
    return 1UL;
}

uint32_t CommandManagement_GetStatus(uint32_t now_ms, command_management_status_t *status)
{
    (void)now_ms;
    *status = (command_management_status_t){
        .motion_allowed           = 1U,
        .permitted_source_mask    = COMMAND_ALL_SOURCE_MASK,
        .motion_revoke_generation = 1UL,
    };
    return 1UL;
}

command_apply_result_t CommandManagement_ApplyIntent(const command_intent_t *intent)
{
    command_apply_result_t result = {COMMAND_OUTCOME_INVALID, 0U, 1UL};

    if (intent->kind == COMMAND_INTENT_REMOTE_DISABLE)
    {
        result.outcome = (CommandManagement_DisableRemoteSource(intent->source) == COMMAND_RESULT_ACCEPTED)
                             ? COMMAND_OUTCOME_RELEASE_ACCEPTED
                             : COMMAND_OUTCOME_GATE_CLOSED;
    }
    else if (intent->kind == COMMAND_INTENT_ACTIVE)
    {
        command_velocity_t command = {
            .linear_x     = intent->linear_x,
            .angular_z    = intent->angular_z,
            .enable       = 1U,
            .source       = intent->source,
            .timestamp_ms = intent->sample_time_ms,
        };
        result.outcome = (CommandManagement_Set(&command) == COMMAND_RESULT_ACCEPTED) ? COMMAND_OUTCOME_ACTIVE_ACCEPTED
                                                                                      : COMMAND_OUTCOME_INVALID;
    }
    return result;
}

uint8_t CommandManagement_IsMotionGateOpen(void)
{
    return 1U;
}

uint8_t SafetyManagement_IsMotionAllowed(void)
{
    return 1U;
}

void ControlService_SetEmergencyStop(uint8_t enable)
{
    estop_set_count++;
    estop_last_value = enable;
}

void SafetyManagement_SetEmergencyStop(uint8_t enable)
{
    ControlService_SetEmergencyStop(enable);
}

uint8_t ControlService_IsEmergencyStop(void)
{
    return 0U;
}

uint8_t SafetyManagement_IsEmergencyStop(void)
{
    return ControlService_IsEmergencyStop();
}

uint8_t ControlService_IsFaultStop(void)
{
    return 0U;
}

line_following_result_t LineFollowing_Enable(uint8_t enable)
{
    (void)enable;
    return LINE_FOLLOWING_RESULT_APPLIED;
}

safety_clear_result_t SafetyManagement_ClearLatchedFaults(uint32_t mask, const safety_clear_input_t *input)
{
    (void)mask;
    (void)input;
    return (safety_clear_result_t){.code = SAFETY_CLEAR_RESULT_APPLIED};
}

void ChassisService_GetState(chassis_service_snapshot_t *state)
{
    if (state != 0)
    {
        *state = (chassis_service_snapshot_t){0};
    }
}

void MotorDriver_GetState(motor_driver_state_t *state)
{
    if (state != 0)
    {
        *state = (motor_driver_state_t){0};
    }
}

void PowerAdcDriver_GetState(power_adc_driver_state_t *state)
{
    *state = fake_adc_state;
}

void PowerOnSelfTest_GetResult(power_on_self_test_result_t *result)
{
    *result = fake_post_result;
}

uint8_t MotorHardwareLayout_MotorEnabled(motor_id_t motor)
{
    return (motor == MOTOR_ID_M2 || motor == MOTOR_ID_M3) ? 1U : 0U;
}

static void RefreshFakePublishModel(void)
{
    fake_publish_model                              = (communication_publish_model_t){0};
    fake_publish_model.post.done                    = fake_post_result.done;
    fake_publish_model.post.error_flags             = fake_post_result.error_flags;
    fake_publish_model.current.invalid_reason_flags = fake_adc_state.invalid_reason_flags;
    fake_publish_model.encoder.anomaly_mask         = fake_encoder_anomaly_mask;
    fake_publish_model.encoder.anomaly_generation   = fake_encoder_anomaly_generation;
    fake_publish_model.control.reset_reason_flags   = 0xA1B2C3D4UL;
    fake_publish_model.imu.online                   = fake_imu_state.online;
    fake_publish_model.imu.calibrated               = fake_imu_state.gyro_calibrated;
    fake_publish_model.imu.sensor_time_valid        = fake_imu_state.sensor_time_valid;
    fake_publish_model.imu.last_error               = fake_imu_state.last_error;
    fake_publish_model.imu.sensor_time              = fake_imu_state.sensor_time;
    fake_publish_model.imu.sample_count             = fake_imu_state.sample_count;
    fake_publish_model.imu.quality_flags            = fake_imu_state.quality_flags;
    fake_publish_model.imu.roll_deg                 = fake_imu_state.roll_deg;
    fake_publish_model.imu.pitch_deg                = fake_imu_state.pitch_deg;
    fake_publish_model.imu.yaw_deg                  = fake_imu_state.yaw_deg;
    fake_publish_model.imu.temperature_c            = fake_imu_state.temperature_c;
    fake_publish_model.imu.quality_counters[0]      = fake_imu_state.spi_error_count;
    fake_publish_model.imu.quality_counters[1]      = fake_imu_state.init_failure_count;
    fake_publish_model.imu.quality_counters[2]      = fake_imu_state.fifo_overflow_count;
    fake_publish_model.imu.quality_counters[3]      = fake_imu_state.timestamp_error_count;
    fake_publish_model.imu.quality_counters[4]      = fake_imu_state.gyro_saturation_count;
    fake_publish_model.imu.quality_counters[5]      = fake_imu_state.accel_anomaly_count;
    fake_publish_model.imu.quality_counters[6]      = fake_imu_state.attitude_invalid_count;
    for (uint8_t index = 0U; index < 3U; ++index)
    {
        fake_publish_model.imu.accel_g[index]  = fake_imu_state.body_accel_g[index];
        fake_publish_model.imu.gyro_dps[index] = fake_imu_state.body_gyro_dps[index];
    }
    for (uint8_t index = 0U; index < 4U; ++index)
    {
        fake_publish_model.imu.quaternion[index] = fake_imu_state.quaternion[index];
    }
}

static void reset_host_uart_state(void)
{
    rx_buffer                       = 0;
    rx_size                         = 0U;
    rx_start_count                  = 0U;
    dma_stop_count                  = 0U;
    tx_dma_count                    = 0U;
    tx_it_count                     = 0U;
    tx_last_size                    = 0U;
    tx_last_uart                    = 0;
    fake_tick                       = 0U;
    fake_imu_state                  = (state_estimation_imu_status_t){0};
    fake_adc_state                  = (power_adc_driver_state_t){0};
    fake_post_result                = (power_on_self_test_result_t){0};
    fake_primask                    = 0U;
    estop_set_count                 = 0UL;
    estop_last_value                = 0U;
    fake_encoder_anomaly_mask       = 0U;
    fake_encoder_anomaly_generation = 0UL;
    overwrite_dma_on_release        = 0U;
    usart3_instance                 = (USART_TypeDef){0};
    usart3_rx_stream                = (DMA_Stream_TypeDef){0};
    hdma_usart3_rx                  = (DMA_HandleTypeDef){.Instance = &usart3_rx_stream};
    huart3                          = (UART_HandleTypeDef){
                                 .Instance = &usart3_instance,
                                 .Init     = {.BaudRate = 115200U},
                                 .hdmarx   = &hdma_usart3_rx,
                                 .gState   = HAL_UART_STATE_READY,
    };
    for (uint16_t i = 0U; i < (uint16_t)sizeof(tx_last_frame); ++i)
    {
        tx_last_frame[i] = 0U;
    }
}
static void require_int(int condition, const char *message)
{
    if (condition == 0)
    {
        (void)printf("FAIL: %s\n", message);
        exit(1);
    }
}

static void feed_valid_estop_frame(uint8_t value)
{
    uint8_t  payload[ROBOT_LINK_PROTOCOL_ESTOP_PAYLOAD_LEN] = {ROBOT_LINK_PROTOCOL_VERSION, value};
    uint8_t  frame[ROBOT_LINK_PROTOCOL_MAX_FRAME]           = {0};
    uint16_t len                                            = UpperProtocol_BuildFrame(UPPER_CMD_ESTOP,
                                            payload,
                                            ROBOT_LINK_PROTOCOL_ESTOP_PAYLOAD_LEN,
                                            frame,
                                            (uint16_t)sizeof(frame));
    for (uint16_t i = 0U; i < len; ++i)
    {
        rx_buffer[i] = frame[i];
    }
    huart3.hdmarx->Instance->NDTR = (uint32_t)(rx_size - len);
}

static uint16_t build_disabled_velocity_frame(uint64_t session_id, uint32_t sequence, uint8_t *frame)
{
    uint8_t payload[23] = {ROBOT_LINK_PROTOCOL_VERSION};

    payload[10] = 2U;
    for (uint8_t index = 0U; index < 8U; ++index)
    {
        payload[11U + index] = (uint8_t)(session_id >> (8U * index));
    }
    for (uint8_t index = 0U; index < 4U; ++index)
    {
        payload[19U + index] = (uint8_t)(sequence >> (8U * index));
    }
    return UpperProtocol_BuildFrame(UPPER_CMD_SET_VELOCITY,
                                    payload,
                                    sizeof(payload),
                                    frame,
                                    ROBOT_LINK_PROTOCOL_MAX_FRAME);
}

static void feed_dma_bytes(const uint8_t *data, uint16_t length)
{
    uint16_t position = (uint16_t)((rx_size - huart3.hdmarx->Instance->NDTR) % rx_size);

    for (uint16_t index = 0U; index < length; ++index)
    {
        rx_buffer[position++] = data[index];
        if (position == rx_size / 2U)
        {
            HostCommunication_OnDmaHalf();
        }
        if (position == rx_size)
        {
            position = 0U;
            HostCommunication_OnDmaFull();
        }
    }
    huart3.hdmarx->Instance->NDTR = (uint32_t)(rx_size - position);
}

static void test_complete_velocity_frame_is_consumed_from_dma(void)
{
    const uint64_t             session = UINT64_C(0x23456789abcdef01);
    uint8_t                    frame[ROBOT_LINK_PROTOCOL_MAX_FRAME];
    host_communication_state_t state;

    reset_host_uart_state();
    fake_tick = 1000U;
    HostCommunication_Init();
    uint16_t size = build_disabled_velocity_frame(session, 1U, frame);
    require_int(size == 28U, "v3 disabled velocity is a complete 28-byte wire frame");
    feed_dma_bytes(frame, size);
    fake_tick += 5U;
    HostCommunication_Update();
    HostCommunication_GetState(&state);
    require_int(state.session.session_id == session && state.session.applied_sequence == 1U,
                "one atomic 28-byte disable must be parsed and applied without fragmentation");
    require_int(state.rx_overwrite_count == 0U && state.rx_resync_restarts == 0U,
                "a complete valid frame is not a DMA overwrite");
}

static void test_valid_dma_burst_and_stable_parse_copy(void)
{
    const uint64_t             session = UINT64_C(0x1234567812345678);
    uint8_t                    frame[ROBOT_LINK_PROTOCOL_MAX_FRAME];
    host_communication_state_t state;

    reset_host_uart_state();
    fake_tick = 1000U;
    HostCommunication_Init();
    for (uint32_t sequence = 1U; sequence <= 16U; ++sequence)
    {
        uint16_t size = build_disabled_velocity_frame(session, sequence, frame);
        feed_dma_bytes(frame, size);
    }
    fake_tick += 38U;
    overwrite_dma_on_release = 1U;
    HostCommunication_Update();
    HostCommunication_GetState(&state);
    require_int(state.session.session_id == session && state.session.applied_sequence == 16U,
                "all 16 complete frames in a bounded 448-byte burst survive DMA storage reuse");
    require_int(state.rx_overwrite_count == 0U && state.rx_resync_restarts == 0U,
                "an unambiguous valid burst is not discarded using outbound frame length");
    require_int(fake_primask == 0U, "DMA snapshot restores interrupt mask");
}

static void test_velocity_frame_crosses_dma_wrap(void)
{
    uint8_t                    noise[100] = {0};
    uint8_t                    frame[ROBOT_LINK_PROTOCOL_MAX_FRAME];
    host_communication_state_t state;

    reset_host_uart_state();
    fake_tick = 1000U;
    HostCommunication_Init();
    for (uint8_t index = 0U; index < 5U; ++index)
    {
        feed_dma_bytes(noise, sizeof(noise));
        fake_tick += 9U;
        HostCommunication_Update();
    }
    require_int(rx_size == 512U, "RX ring provides margin above normal 5ms polling");
    feed_dma_bytes(frame, build_disabled_velocity_frame(19U, 1U, frame));
    fake_tick += 5U;
    HostCommunication_Update();
    HostCommunication_GetState(&state);
    require_int(state.rx_dma_full_count == 1U, "control frame crossed the circular DMA boundary");
    require_int(state.session.session_id == 19U && state.session.applied_sequence == 1U,
                "a complete frame spanning the DMA wrap is applied once");
    require_int(state.rx_overwrite_count == 0U, "ordinary DMA wrap is not an overwrite");
}

static void test_ambiguous_full_dma_laps_are_discarded_then_recover(void)
{
    uint8_t                    noise[512] = {0};
    uint8_t                    frame[ROBOT_LINK_PROTOCOL_MAX_FRAME];
    host_communication_state_t state;
    host_communication_state_t before;

    reset_host_uart_state();
    fake_tick = 1000U;
    HostCommunication_Init();
    /* Session initialization is intentionally idempotent across link restarts. */
    HostCommunication_GetState(&before);
    feed_dma_bytes(noise, sizeof(noise));
    fake_tick += 45U;
    HostCommunication_Update();
    HostCommunication_GetState(&state);
    require_int(state.rx_overwrite_count == 1U && state.rx_resync_restarts == 1U,
                "a possible full lap is detected even when NDTR returns to the same position");

    feed_dma_bytes(noise, sizeof(noise));
    feed_dma_bytes(noise, sizeof(noise));
    uint16_t size = build_disabled_velocity_frame(23U, 1U, frame);
    feed_dma_bytes(frame, size);
    fake_tick += 95U;
    HostCommunication_Update();
    HostCommunication_GetState(&state);
    require_int(state.rx_overwrite_count == 2U && state.session.session_id == before.session.session_id
                    && state.session.generation == before.session.generation
                    && state.session.received_sequence == before.session.received_sequence
                    && state.session.applied_sequence == before.session.applied_sequence,
                "data after multiple unobserved laps cannot manufacture a fresh command");
    size = build_disabled_velocity_frame(23U, 2U, frame);
    feed_dma_bytes(frame, size);
    fake_tick += 5U;
    HostCommunication_Update();
    HostCommunication_GetState(&state);
    require_int(state.session.session_id == 23U && state.session.applied_sequence == 2U,
                "a new complete command is accepted after DMA resynchronization");
}

static void test_dma_poll_budget_tracks_baud_and_tick_wrap(void)
{
    uint8_t                    frame[ROBOT_LINK_PROTOCOL_MAX_FRAME];
    host_communication_state_t state;
    host_communication_state_t before;

    reset_host_uart_state();
    huart3.Init.BaudRate = 0U;
    require_int(HostCommunication_Init() == 0U && rx_start_count == 0U,
                "an unknown UART baud rate cannot create a fictitious safe polling budget");

    reset_host_uart_state();
    huart3.Init.BaudRate = 230400U;
    fake_tick            = 1000U;
    HostCommunication_Init();
    HostCommunication_GetState(&before);
    feed_dma_bytes(frame, build_disabled_velocity_frame(29U, 1U, frame));
    fake_tick += 20U;
    HostCommunication_Update();
    HostCommunication_GetState(&state);
    require_int(state.rx_overwrite_count == 1U && state.session.session_id == before.session.session_id
                    && state.session.generation == before.session.generation
                    && state.session.received_sequence == before.session.received_sequence
                    && state.session.applied_sequence == before.session.applied_sequence,
                "higher baud shortens the conservative full-lap polling budget");

    reset_host_uart_state();
    fake_tick = UINT32_MAX - 2U;
    HostCommunication_Init();
    feed_dma_bytes(frame, build_disabled_velocity_frame(31U, 1U, frame));
    fake_tick = 2U;
    HostCommunication_Update();
    HostCommunication_GetState(&state);
    require_int(state.session.session_id == 31U && state.session.applied_sequence == 1U,
                "tick rollover preserves the bounded elapsed polling calculation");
}

static void test_dma_callbacks_and_valid_frame_timestamp(void)
{
    host_communication_state_t state;

    reset_host_uart_state();
    fake_tick = 1000U;
    HostCommunication_Init();
    require_int(rx_start_count == 1U, "upper uart starts rx dma");

    HostCommunication_OnDmaHalf();
    HostCommunication_OnDmaFull();
    feed_valid_estop_frame(1U);
    HostCommunication_Update();
    HostCommunication_GetState(&state);
    require_int(state.rx_dma_half_count == 1U, "rx half count");
    require_int(state.rx_dma_full_count == 1U, "rx full count");
    require_int(state.last_valid_frame_ms == 1000U, "last valid frame timestamp");
}

static void test_remote_estop_is_set_only(void)
{
    communication_operation_request_t operation;

    reset_host_uart_state();
    HostCommunication_Init();
    feed_valid_estop_frame(0U);
    HostCommunication_Update();
    require_int(estop_set_count == 0UL, "remote ESTOP=0 cannot clear local emergency stop");

    reset_host_uart_state();
    HostCommunication_Init();
    feed_valid_estop_frame(1U);
    HostCommunication_Update();
    require_int(estop_set_count == 0UL, "Communication does not apply ESTOP business state directly");
    require_int(HostCommunication_TakeOperation(fake_tick, &operation) != 0U,
                "remote ESTOP=1 queues an App-owned operation");
    require_int(operation.kind == COMMUNICATION_OPERATION_ESTOP && operation.value == 1U,
                "queued operation preserves ESTOP request");
}

static void test_parser_timeout_and_uart_error_restart_dma(void)
{
    host_communication_state_t state;
    uint32_t                   starts_before;

    reset_host_uart_state();
    HostCommunication_Init();
    rx_buffer[0]                  = ROBOT_LINK_PROTOCOL_HEAD_0;
    huart3.hdmarx->Instance->NDTR = (uint32_t)(rx_size - 1U);
    HostCommunication_Update();
    fake_tick += 101U;
    HostCommunication_Update();
    HostCommunication_GetState(&state);
    require_int(state.rx_timeout_resets != 0U, "parser timeout reset counted");
    require_int(state.rx_resync_restarts != 0U, "parser timeout restarts dma");

    starts_before       = rx_start_count;
    huart3.Instance->SR = UART_FLAG_ORE | UART_FLAG_NE;
    HostCommunication_OnUartError();
    HostCommunication_GetState(&state);
    require_int(dma_stop_count != 0U, "uart error stops dma");
    require_int(rx_start_count == starts_before + 1U, "uart error restarts dma");
    require_int(state.uart_errors != 0U, "uart error counted");
    require_int(huart3.Instance->SR == 0U, "uart flags cleared");
}

static void test_status_uses_interrupt_tx_without_usart3_tx_dma(void)
{
    host_communication_state_t state;

    reset_host_uart_state();
    fake_tick     = 1000U;
    huart3.hdmatx = 0;
    huart3.gState = HAL_UART_STATE_READY;

    HostCommunication_Init();
    HostCommunication_Update();
    HostCommunication_GetState(&state);

    require_int(tx_dma_count == 0U, "usart3 status must not use tx dma when no tx dma is configured");
    require_int(tx_it_count == 1U, "usart3 status uses interrupt tx");
    require_int(tx_last_uart == &huart3, "status frame sent on usart3");
    require_int(tx_last_size > 5U, "status frame has bytes");
    require_int(tx_last_frame[0] == ROBOT_LINK_PROTOCOL_HEAD_0, "status frame head0");
    require_int(tx_last_frame[1] == ROBOT_LINK_PROTOCOL_HEAD_1, "status frame head1");
    require_int(tx_last_frame[3] == UPPER_CMD_STATUS, "status frame cmd");
    require_int(state.tx_frames == 0U, "status is not completed before callback");
    huart3.gState = HAL_UART_STATE_READY;
    HostCommunication_OnTxComplete();
    HostCommunication_GetState(&state);
    require_int(state.tx_frames == 1U, "status completion counted in callback");
}

static void test_anomaly_delivery_requires_physical_status_completion(void)
{
    uint32_t generation = 0UL;

    reset_host_uart_state();
    fake_tick                       = 1000U;
    fake_encoder_anomaly_mask       = 0x02U;
    fake_encoder_anomaly_generation = 17UL;
    huart3.gState                   = HAL_UART_STATE_READY;
    HostCommunication_Init();
    HostCommunication_Update();
    require_int(HostCommunication_TakeAnomalyDelivery(&generation) == 0U,
                "queued STATUS does not acknowledge anomaly delivery");
    huart3.gState = HAL_UART_STATE_READY;
    HostCommunication_OnTxComplete();
    require_int(HostCommunication_TakeAnomalyDelivery(&generation) != 0U && generation == 17UL,
                "physical STATUS completion publishes its anomaly generation");
    require_int(HostCommunication_TakeAnomalyDelivery(&generation) == 0U, "anomaly delivery event is consumed once");
}

static void test_status_and_imu_share_async_priority_queue(void)
{
    host_communication_state_t state;

    reset_host_uart_state();
    fake_tick                    = 1000U;
    fake_imu_state.online        = 1U;
    fake_imu_state.temperature_c = 23.0f;
    huart3.gState                = HAL_UART_STATE_READY;

    HostCommunication_Init();
    HostCommunication_Update();
    HostCommunication_GetState(&state);
    require_int(tx_it_count == 1U, "only status starts while uart is busy");
    require_int(tx_last_frame[3] == UPPER_CMD_STATUS, "status has priority over imu");
    require_int(state.tx_frames == 0U, "no async frame completes without callback");

    huart3.gState = HAL_UART_STATE_READY;
    HostCommunication_OnTxComplete();
    HostCommunication_GetState(&state);
    require_int(tx_it_count == 2U, "diagnostic starts after status completion");
    require_int(tx_last_frame[3] == UPPER_CMD_DIAGNOSTIC, "queued diagnostic follows status");
    require_int(tx_last_frame[2] == ROBOT_LINK_PROTOCOL_CMD_LEN(ROBOT_LINK_PROTOCOL_DIAGNOSTIC_PAYLOAD_LEN),
                "diagnostic frame carries fixed payload length");
    require_int(tx_last_frame[4] == ROBOT_LINK_PROTOCOL_VERSION, "diagnostic frame carries protocol version");
    require_int(tx_last_frame[5] == ROBOT_LINK_PROTOCOL_DIAGNOSTIC_SCHEMA_VERSION,
                "diagnostic frame carries schema version");
    require_int(tx_last_frame[24] == 0xD4U && tx_last_frame[27] == 0xA1U,
                "diagnostic frame carries reset reason flags");
    require_int(state.tx_frames == 1U, "status completion counted");

    huart3.gState = HAL_UART_STATE_READY;
    HostCommunication_OnTxComplete();
    HostCommunication_GetState(&state);
    require_int(tx_it_count == 3U, "imu starts after diagnostic completion");
    require_int(tx_last_frame[3] == UPPER_CMD_IMU_STATUS, "queued imu follows diagnostic");
    require_int(tx_last_frame[4U + ROBOT_LINK_PROTOCOL_IMU_STATUS_PAYLOAD_LEN - 1U] == 23U,
                "imu temperature is direct signed celsius");
    require_int(state.tx_frames == 2U, "diagnostic completion counted");

    huart3.gState = HAL_UART_STATE_READY;
    HostCommunication_OnTxComplete();
    HostCommunication_GetState(&state);
    require_int(state.tx_frames == 3U, "all async frames complete");
}

static void test_uart_error_releases_active_tx_and_continues_queue(void)
{
    host_communication_state_t state;

    reset_host_uart_state();
    fake_tick             = 1000U;
    fake_imu_state.online = 1U;
    huart3.gState         = HAL_UART_STATE_READY;

    HostCommunication_Init();
    HostCommunication_Update();
    require_int(tx_it_count == 1U, "status starts before injected UART error");
    huart3.gState = HAL_UART_STATE_READY;
    HostCommunication_OnUartError();
    HostCommunication_GetState(&state);
    require_int(state.tx_busy_drops == 1U, "failed active frame is dropped once");
    require_int(tx_it_count == 2U, "queued IMU starts after UART error recovery");
    require_int(tx_last_frame[3] == UPPER_CMD_DIAGNOSTIC, "UART error recovery advances to diagnostic frame");
}

static void test_full_queue_drops_imu_for_status_priority(void)
{
    host_communication_state_t state;

    reset_host_uart_state();
    fake_tick             = 1000U;
    fake_imu_state.online = 1U;
    huart3.gState         = HAL_UART_STATE_READY;
    HostCommunication_Init();
    HostCommunication_Update();

    fake_tick = 1020U;
    HostCommunication_Update();
    fake_tick = 1040U;
    HostCommunication_Update();
    fake_tick = 1050U;
    HostCommunication_Update();
    HostCommunication_GetState(&state);
    require_int(state.tx_busy_drops == 0U, "duplicate pending IMU is coalesced without starving status");

    huart3.gState = HAL_UART_STATE_READY;
    HostCommunication_OnTxComplete();
    require_int(tx_last_frame[3] == UPPER_CMD_STATUS, "queued status is selected before older IMU telemetry");
}

int main(void)
{
    test_complete_velocity_frame_is_consumed_from_dma();
    test_valid_dma_burst_and_stable_parse_copy();
    test_velocity_frame_crosses_dma_wrap();
    test_ambiguous_full_dma_laps_are_discarded_then_recover();
    test_dma_poll_budget_tracks_baud_and_tick_wrap();
    test_status_uses_interrupt_tx_without_usart3_tx_dma();
    test_anomaly_delivery_requires_physical_status_completion();
    test_status_and_imu_share_async_priority_queue();
    test_uart_error_releases_active_tx_and_continues_queue();
    test_full_queue_drops_imu_for_status_priority();
    test_dma_callbacks_and_valid_frame_timestamp();
    test_parser_timeout_and_uart_error_restart_dma();
    test_remote_estop_is_set_only();
    (void)printf("PASS: upper uart host tests\n");
    return 0;
}
