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
UART_HandleTypeDef huart3 = {.Instance = &usart3_instance, .hdmarx = &hdma_usart3_rx, .gState = HAL_UART_STATE_READY};

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

uint8_t CommandManagement_RefreshSource(command_source_t source, uint32_t now_ms)
{
    (void)source;
    (void)now_ms;
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

void LineFollowing_Enable(uint8_t enable)
{
    (void)enable;
}

void SafetyManagement_ClearLatchedFaults(uint32_t mask)
{
    (void)mask;
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
    rx_buffer        = 0;
    rx_size          = 0U;
    rx_start_count   = 0U;
    dma_stop_count   = 0U;
    tx_dma_count     = 0U;
    tx_it_count      = 0U;
    tx_last_size     = 0U;
    tx_last_uart     = 0;
    fake_tick        = 0U;
    fake_imu_state   = (state_estimation_imu_status_t){0};
    fake_adc_state   = (power_adc_driver_state_t){0};
    fake_post_result = (power_on_self_test_result_t){0};
    fake_primask     = 0U;
    estop_set_count  = 0UL;
    estop_last_value = 0U;
    usart3_instance  = (USART_TypeDef){0};
    usart3_rx_stream = (DMA_Stream_TypeDef){0};
    hdma_usart3_rx   = (DMA_HandleTypeDef){.Instance = &usart3_rx_stream};
    huart3 =
        (UART_HandleTypeDef){.Instance = &usart3_instance, .hdmarx = &hdma_usart3_rx, .gState = HAL_UART_STATE_READY};
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
    reset_host_uart_state();
    HostCommunication_Init();
    feed_valid_estop_frame(0U);
    HostCommunication_Update();
    require_int(estop_set_count == 0UL, "remote ESTOP=0 cannot clear local emergency stop");

    reset_host_uart_state();
    HostCommunication_Init();
    feed_valid_estop_frame(1U);
    HostCommunication_Update();
    require_int(estop_set_count == 1UL && estop_last_value == 1U, "remote ESTOP=1 sets emergency stop");
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
    for (uint8_t i = 0U; i < 21U; ++i)
    {
        HostCommunication_Update();
    }
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
    test_status_uses_interrupt_tx_without_usart3_tx_dma();
    test_status_and_imu_share_async_priority_queue();
    test_uart_error_releases_active_tx_and_continues_queue();
    test_full_queue_drops_imu_for_status_priority();
    test_dma_callbacks_and_valid_frame_timestamp();
    test_parser_timeout_and_uart_error_restart_dma();
    test_remote_estop_is_set_only();
    (void)printf("PASS: upper uart host tests\n");
    return 0;
}
