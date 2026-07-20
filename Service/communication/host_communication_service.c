#include "host_communication_service.h"
#include "remote_command_dispatcher.h"
#include "communication_session_tracker.h"
#include "frame_stream_parser.h"
#include "telemetry_encoder.h"
#include "platform_critical.h"
#include "platform_time.h"
#include "host_uart_transport.h"

#include "robot_link_protocol.h"

#include <string.h>

#define UPPER_UART_RX_BUFFER_SIZE    128U
#define UPPER_UART_TX_QUEUE_CAPACITY 4U
#define UPPER_UART_TX_SLOT_INVALID   0xFFU

typedef enum
{
    UPPER_TX_PRIORITY_IMU        = 0,
    UPPER_TX_PRIORITY_DIAGNOSTIC = 1,
    UPPER_TX_PRIORITY_HELLO      = 2,
    UPPER_TX_PRIORITY_STATUS     = 3
} upper_tx_priority_t;

typedef struct
{
    uint8_t  data[ROBOT_LINK_PROTOCOL_MAX_FRAME];
    uint16_t length;
    uint32_t sequence;
    uint8_t  priority;
    uint8_t  used;
} upper_tx_slot_t;

static uint8_t                           upper_rx_dma_buffer[UPPER_UART_RX_BUFFER_SIZE] __attribute__((aligned(4)));
static uint16_t                          upper_rx_read_pos;
static frame_parser_t                    upper_parser;
static uint8_t                           upper_tx_frame[ROBOT_LINK_PROTOCOL_MAX_FRAME];
static uint32_t                          upper_last_status_ms;
static uint32_t                          upper_last_rx_timestamp_ms;
static communication_config_t            upper_config;
static host_communication_state_t        upper_state;
static uint8_t                           upper_parser_idle_cycles;
static uint16_t                          upper_last_write_pos;
static uint8_t                           upper_imu_tx_frame[ROBOT_LINK_PROTOCOL_MAX_FRAME];
static uint32_t                          upper_last_imu_status_ms;
static uint8_t                           upper_diagnostic_tx_frame[ROBOT_LINK_PROTOCOL_MAX_FRAME];
static uint32_t                          upper_last_diagnostic_ms;
static upper_tx_slot_t                   upper_tx_queue[UPPER_UART_TX_QUEUE_CAPACITY];
static uint8_t                           upper_tx_queue_count;
static uint8_t                           upper_tx_active_slot;
static uint32_t                          upper_tx_sequence;
static uint8_t                           upper_hello_pending;
static communication_firmware_identity_t upper_identity;
static uint8_t                           upper_hello_tx_frame[ROBOT_LINK_PROTOCOL_MAX_FRAME];

#define UPPER_PARSER_TIMEOUT_CYCLES 20U /* 20 × 5ms = 100ms 无字节则重置解析器 */

static void HostCommunication_ResetParser(void)
{
    FrameParser_Reset(&upper_parser);
}

static uint8_t HostCommunication_SelectNextTxLocked(void)
{
    uint8_t selected = UPPER_UART_TX_SLOT_INVALID;

    for (uint8_t i = 0U; i < UPPER_UART_TX_QUEUE_CAPACITY; ++i)
    {
        if (upper_tx_queue[i].used == 0U || i == upper_tx_active_slot)
        {
            continue;
        }
        if (selected == UPPER_UART_TX_SLOT_INVALID || upper_tx_queue[i].priority > upper_tx_queue[selected].priority
            || (upper_tx_queue[i].priority == upper_tx_queue[selected].priority
                && upper_tx_queue[i].sequence < upper_tx_queue[selected].sequence))
        {
            selected = i;
        }
    }
    return selected;
}

static uint8_t HostCommunication_DropQueuedImuLocked(void)
{
    uint8_t selected = UPPER_UART_TX_SLOT_INVALID;

    for (uint8_t i = 0U; i < UPPER_UART_TX_QUEUE_CAPACITY; ++i)
    {
        if (upper_tx_queue[i].used == 0U || i == upper_tx_active_slot
            || upper_tx_queue[i].priority != UPPER_TX_PRIORITY_IMU)
        {
            continue;
        }
        if (selected == UPPER_UART_TX_SLOT_INVALID || upper_tx_queue[i].sequence < upper_tx_queue[selected].sequence)
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

static uint8_t HostCommunication_HasQueuedImuLocked(void)
{
    for (uint8_t i = 0U; i < UPPER_UART_TX_QUEUE_CAPACITY; ++i)
    {
        if (upper_tx_queue[i].used != 0U && i != upper_tx_active_slot
            && upper_tx_queue[i].priority == UPPER_TX_PRIORITY_IMU)
        {
            return 1U;
        }
    }
    return 0U;
}

static void HostCommunication_TryStartTxLocked(void)
{
    uint8_t            selected;
    transport_status_t status;

    if (upper_tx_active_slot != UPPER_UART_TX_SLOT_INVALID)
    {
        return;
    }
    selected = HostCommunication_SelectNextTxLocked();
    if (selected == UPPER_UART_TX_SLOT_INVALID)
    {
        return;
    }
    status = HostUartTransport_TransmitAsync(upper_tx_queue[selected].data, upper_tx_queue[selected].length);
    if (status == TRANSPORT_STATUS_OK)
    {
        upper_tx_active_slot = selected;
    }
    else if (status != TRANSPORT_STATUS_BUSY)
    {
        if (upper_tx_queue[selected].data[3] == UPPER_CMD_HELLO)
        {
            upper_hello_pending = 1U;
        }
        upper_tx_queue[selected].used = 0U;
        upper_tx_queue_count--;
        upper_state.tx_busy_drops++;
    }
}

static uint8_t HostCommunication_EnqueueTx(const uint8_t *frame, uint16_t frame_len, upper_tx_priority_t priority)
{
    uint32_t primask;
    uint8_t  selected = UPPER_UART_TX_SLOT_INVALID;

    if (frame == 0 || frame_len == 0U || frame_len > ROBOT_LINK_PROTOCOL_MAX_FRAME)
    {
        return 0U;
    }
    primask = PlatformCritical_Enter();
    if (priority == UPPER_TX_PRIORITY_IMU && HostCommunication_HasQueuedImuLocked() != 0U)
    {
        PlatformCritical_Exit(primask);
        return 0U;
    }
    if (upper_tx_queue_count >= UPPER_UART_TX_QUEUE_CAPACITY)
    {
        if (priority == UPPER_TX_PRIORITY_IMU || HostCommunication_DropQueuedImuLocked() == 0U)
        {
            upper_state.tx_busy_drops++;
            PlatformCritical_Exit(primask);
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
        PlatformCritical_Exit(primask);
        return 0U;
    }
    memcpy(upper_tx_queue[selected].data, frame, frame_len);
    upper_tx_queue[selected].length   = frame_len;
    upper_tx_queue[selected].priority = (uint8_t)priority;
    upper_tx_queue[selected].sequence = upper_tx_sequence++;
    upper_tx_queue[selected].used     = 1U;
    upper_tx_queue_count++;
    HostCommunication_TryStartTxLocked();
    PlatformCritical_Exit(primask);
    return 1U;
}

static void HostCommunication_RestartRxDma(uint8_t count_restart)
{
    HostUartTransport_RestartRx(upper_rx_dma_buffer, UPPER_UART_RX_BUFFER_SIZE);
    upper_rx_read_pos        = 0U;
    upper_last_write_pos     = 0U;
    upper_parser_idle_cycles = 0U;
    HostCommunication_ResetParser();
    if (count_restart != 0U)
    {
        upper_state.rx_resync_restarts++;
    }
}

static void HostCommunication_ProcessByte(uint8_t byte)
{
    protocol_frame_t      frame;
    frame_parser_result_t result = FrameParser_Push(&upper_parser, byte, &frame);

    if (result == FRAME_PARSER_FRAME)
    {
        uint32_t now_ms = PlatformTime_TaskNowMs();
        if (RemoteCommandDispatcher_Handle(COMMUNICATION_LINK_UPPER, &frame, now_ms) == REMOTE_ACTION_REQUEST_INFO)
        {
            upper_hello_pending = 1U;
        }
        upper_last_rx_timestamp_ms      = now_ms;
        upper_state.last_valid_frame_ms = now_ms;
    }
    else if (result == FRAME_PARSER_CHECKSUM_ERROR)
    {
        upper_state.rx_checksum_errors++;
    }
}

static void HostCommunication_PollRx(void)
{
    uint16_t write_pos;

    write_pos = HostUartTransport_GetRxWritePosition(UPPER_UART_RX_BUFFER_SIZE);
    if (write_pos >= UPPER_UART_RX_BUFFER_SIZE)
    {
        write_pos = 0U;
    }

    if (upper_rx_read_pos == write_pos)
    {
        /* 无新字节：若解析器处于中间状态，超时后重置 */
        if (FrameParser_IsIdle(&upper_parser) == 0U)
        {
            upper_parser_idle_cycles++;
            if (upper_parser_idle_cycles >= UPPER_PARSER_TIMEOUT_CYCLES)
            {
                upper_state.rx_timeout_resets++;
                HostCommunication_RestartRxDma(1U);
            }
        }
        return;
    }

    upper_parser_idle_cycles = 0U;
    {
        uint16_t unread   = (write_pos >= upper_rx_read_pos)
                                ? (uint16_t)(write_pos - upper_rx_read_pos)
                                : (uint16_t)(UPPER_UART_RX_BUFFER_SIZE - upper_rx_read_pos + write_pos);
        uint16_t advanced = (write_pos >= upper_last_write_pos)
                                ? (uint16_t)(write_pos - upper_last_write_pos)
                                : (uint16_t)(UPPER_UART_RX_BUFFER_SIZE - upper_last_write_pos + write_pos);
        if (advanced > 0U && unread >= (UPPER_UART_RX_BUFFER_SIZE - ROBOT_LINK_PROTOCOL_MAX_FRAME))
        {
            upper_state.rx_overwrite_count++;
            HostCommunication_RestartRxDma(1U);
            return;
        }
        upper_last_write_pos = write_pos;
    }
    while (upper_rx_read_pos != write_pos)
    {
        HostCommunication_ProcessByte(upper_rx_dma_buffer[upper_rx_read_pos]);
        upper_rx_read_pos++;
        if (upper_rx_read_pos >= UPPER_UART_RX_BUFFER_SIZE)
        {
            upper_rx_read_pos = 0U;
        }
    }
}

static void HostCommunication_SendStatus(uint32_t now_ms, const communication_publish_model_t *snapshot)
{
    uint16_t frame_len;

    if ((now_ms - upper_last_status_ms) < upper_config.host_status_period_ms)
    {
        return;
    }

    frame_len =
        TelemetryEncoder_BuildStatus(snapshot, COMMUNICATION_LINK_UPPER, upper_tx_frame, sizeof(upper_tx_frame));
    if (frame_len > 0U)
    {
        if (HostCommunication_EnqueueTx(upper_tx_frame, frame_len, UPPER_TX_PRIORITY_STATUS) != 0U)
        {
            upper_last_status_ms = now_ms;
        }
    }
}

uint8_t HostCommunication_Init(const communication_config_t *config, const communication_firmware_identity_t *identity)
{
    if (config == 0 || identity == 0 || config->host_status_period_ms == 0U || config->host_imu_status_period_ms == 0U
        || config->host_diagnostic_period_ms == 0U)
    {
        return 0U;
    }
    upper_config             = *config;
    upper_identity           = *identity;
    upper_rx_read_pos        = 0U;
    upper_last_write_pos     = 0U;
    upper_last_status_ms     = 0U;
    upper_last_imu_status_ms = 0U;
    upper_last_diagnostic_ms = 0U;
    upper_state              = (host_communication_state_t){0};
    memset(upper_tx_queue, 0, sizeof(upper_tx_queue));
    upper_tx_queue_count       = 0U;
    upper_tx_active_slot       = UPPER_UART_TX_SLOT_INVALID;
    upper_tx_sequence          = 0UL;
    upper_hello_pending        = 0U;
    upper_last_rx_timestamp_ms = 0U;
    HostCommunication_ResetParser();
    CommunicationSessionTracker_Init();
    HostUartTransport_StartRx(upper_rx_dma_buffer, UPPER_UART_RX_BUFFER_SIZE);
    return 1U;
}

static void HostCommunication_SendHello(const communication_publish_model_t *snapshot)
{
    uint16_t frame_len;

    if (upper_hello_pending == 0U)
    {
        return;
    }
    frame_len = TelemetryEncoder_BuildHello(&upper_identity,
                                            snapshot->parameter_identity_crc32,
                                            upper_hello_tx_frame,
                                            sizeof(upper_hello_tx_frame));
    if (frame_len > 0U && HostCommunication_EnqueueTx(upper_hello_tx_frame, frame_len, UPPER_TX_PRIORITY_HELLO) != 0U)
    {
        upper_hello_pending = 0U;
    }
}

static void HostCommunication_SendDiagnostic(uint32_t now_ms, const communication_publish_model_t *snapshot)
{
    uint16_t frame_len;

    if ((now_ms - upper_last_diagnostic_ms) < upper_config.host_diagnostic_period_ms)
    {
        return;
    }

    frame_len =
        TelemetryEncoder_BuildDiagnostic(snapshot, upper_diagnostic_tx_frame, sizeof(upper_diagnostic_tx_frame));
    if (frame_len > 0U
        && HostCommunication_EnqueueTx(upper_diagnostic_tx_frame, frame_len, UPPER_TX_PRIORITY_DIAGNOSTIC) != 0U)
    {
        upper_last_diagnostic_ms = now_ms;
    }
}

static void HostCommunication_SendImuStatus(uint32_t now_ms, const communication_publish_model_t *snapshot)
{
    uint16_t frame_len;

    if ((now_ms - upper_last_imu_status_ms) < upper_config.host_imu_status_period_ms)
    {
        return;
    }

    frame_len = TelemetryEncoder_BuildImu(snapshot, upper_imu_tx_frame, sizeof(upper_imu_tx_frame));
    if (frame_len > 0U)
    {
        if (HostCommunication_EnqueueTx(upper_imu_tx_frame, frame_len, UPPER_TX_PRIORITY_IMU) != 0U)
        {
            upper_last_imu_status_ms = now_ms;
        }
    }
}

void HostCommunication_Update(const communication_publish_model_t *publish_model)
{
    uint32_t now_ms = PlatformTime_TaskNowMs();

    HostCommunication_PollRx();
    if (publish_model != 0)
    {
        HostCommunication_SendStatus(now_ms, publish_model);
        HostCommunication_SendHello(publish_model);
        HostCommunication_SendDiagnostic(now_ms, publish_model);
        HostCommunication_SendImuStatus(now_ms, publish_model);
    }
}

void HostCommunication_GetState(host_communication_state_t *state)
{
    if (state != 0)
    {
        *state = upper_state;
        CommunicationSessionTracker_GetSnapshot(COMMUNICATION_LINK_UPPER, &state->session);
    }
}

void HostCommunication_OnUartError(void)
{
    uint32_t primask;

    upper_state.uart_errors++;
    HostCommunication_RestartRxDma(1U);
    primask = PlatformCritical_Enter();
    if (upper_tx_active_slot != UPPER_UART_TX_SLOT_INVALID && upper_tx_queue[upper_tx_active_slot].used != 0U)
    {
        if (upper_tx_queue[upper_tx_active_slot].data[3] == UPPER_CMD_HELLO)
        {
            upper_hello_pending = 1U;
        }
        upper_tx_queue[upper_tx_active_slot].used = 0U;
        upper_tx_queue_count--;
        upper_state.tx_busy_drops++;
    }
    upper_tx_active_slot = UPPER_UART_TX_SLOT_INVALID;
    HostCommunication_TryStartTxLocked();
    PlatformCritical_Exit(primask);
}

uint32_t HostCommunication_GetLastRxTimestamp(void)
{
    return upper_last_rx_timestamp_ms;
}

void HostCommunication_OnDmaHalf(void)
{
    upper_state.rx_dma_half_count++;
}

void HostCommunication_OnDmaFull(void)
{
    upper_state.rx_dma_full_count++;
}

void HostCommunication_OnTxComplete(void)
{
    uint32_t primask = PlatformCritical_Enter();
    if (upper_tx_active_slot != UPPER_UART_TX_SLOT_INVALID && upper_tx_queue[upper_tx_active_slot].used != 0U)
    {
        upper_tx_queue[upper_tx_active_slot].used = 0U;
        upper_tx_queue_count--;
        upper_state.tx_frames++;
    }
    upper_tx_active_slot = UPPER_UART_TX_SLOT_INVALID;
    HostCommunication_TryStartTxLocked();
    PlatformCritical_Exit(primask);
}
