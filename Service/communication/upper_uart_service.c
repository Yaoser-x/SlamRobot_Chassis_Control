#include "upper_uart_service.h"
#include "communication_command_router.h"
#include "framed_stream_parser.h"
#include "system_snapshot_service.h"
#include "telemetry_frame_builder.h"
#include "platform_critical.h"
#include "platform_time.h"
#include "upper_uart_transport.h"

#include "control_config.h"
#include "upper_protocol.h"

#include <string.h>

#define UPPER_UART_RX_BUFFER_SIZE    128U
#define UPPER_UART_TX_QUEUE_CAPACITY 4U
#define UPPER_UART_TX_SLOT_INVALID   0xFFU

typedef enum
{
    UPPER_TX_PRIORITY_IMU        = 0,
    UPPER_TX_PRIORITY_DIAGNOSTIC = 1,
    UPPER_TX_PRIORITY_STATUS     = 2
} upper_tx_priority_t;

typedef struct
{
    uint8_t  data[UPPER_PROTOCOL_MAX_FRAME];
    uint16_t length;
    uint32_t sequence;
    uint8_t  priority;
    uint8_t  used;
} upper_tx_slot_t;

static uint8_t                    upper_rx_dma_buffer[UPPER_UART_RX_BUFFER_SIZE] __attribute__((aligned(4)));
static uint16_t                   upper_rx_read_pos;
static frame_parser_t             upper_parser;
static uint8_t                    upper_tx_frame[UPPER_PROTOCOL_MAX_FRAME];
static uint32_t                   upper_last_status_ms;
static uint32_t                   upper_last_rx_timestamp_ms;
static upper_uart_service_state_t upper_state;
static uint8_t                    upper_parser_idle_cycles;
static uint16_t                   upper_last_write_pos;
static uint8_t                    upper_imu_tx_frame[UPPER_PROTOCOL_MAX_FRAME];
static uint32_t                   upper_last_imu_status_ms;
static uint8_t                    upper_diagnostic_tx_frame[UPPER_PROTOCOL_MAX_FRAME];
static uint32_t                   upper_last_diagnostic_ms;
static upper_tx_slot_t            upper_tx_queue[UPPER_UART_TX_QUEUE_CAPACITY];
static uint8_t                    upper_tx_queue_count;
static uint8_t                    upper_tx_active_slot;
static uint32_t                   upper_tx_sequence;

#define UPPER_PARSER_TIMEOUT_CYCLES 20U /* 20 × 5ms = 100ms 无字节则重置解析器 */

static void UpperUartService_ResetParser(void)
{
    FrameParser_Reset(&upper_parser);
}

static uint8_t UpperUartService_SelectNextTxLocked(void)
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

static uint8_t UpperUartService_DropQueuedImuLocked(void)
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

static void UpperUartService_TryStartTxLocked(void)
{
    uint8_t            selected;
    transport_status_t status;

    if (upper_tx_active_slot != UPPER_UART_TX_SLOT_INVALID)
    {
        return;
    }
    selected = UpperUartService_SelectNextTxLocked();
    if (selected == UPPER_UART_TX_SLOT_INVALID)
    {
        return;
    }
    status = UpperUartTransport_TransmitAsync(upper_tx_queue[selected].data, upper_tx_queue[selected].length);
    if (status == TRANSPORT_STATUS_OK)
    {
        upper_tx_active_slot = selected;
    }
    else if (status != TRANSPORT_STATUS_BUSY)
    {
        upper_tx_queue[selected].used = 0U;
        upper_tx_queue_count--;
        upper_state.tx_busy_drops++;
    }
}

static uint8_t UpperUartService_EnqueueTx(const uint8_t *frame, uint16_t frame_len, upper_tx_priority_t priority)
{
    uint32_t primask;
    uint8_t  selected = UPPER_UART_TX_SLOT_INVALID;

    if (frame == 0 || frame_len == 0U || frame_len > UPPER_PROTOCOL_MAX_FRAME)
    {
        return 0U;
    }
    primask = PlatformCritical_Enter();
    if (upper_tx_queue_count >= UPPER_UART_TX_QUEUE_CAPACITY)
    {
        if (priority == UPPER_TX_PRIORITY_IMU || UpperUartService_DropQueuedImuLocked() == 0U)
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
    UpperUartService_TryStartTxLocked();
    PlatformCritical_Exit(primask);
    return 1U;
}

static void UpperUartService_RestartRxDma(uint8_t count_restart)
{
    UpperUartTransport_RestartRx(upper_rx_dma_buffer, UPPER_UART_RX_BUFFER_SIZE);
    upper_rx_read_pos        = 0U;
    upper_last_write_pos     = 0U;
    upper_parser_idle_cycles = 0U;
    UpperUartService_ResetParser();
    if (count_restart != 0U)
    {
        upper_state.rx_resync_restarts++;
    }
}

static void UpperUartService_ProcessByte(uint8_t byte)
{
    protocol_frame_t      frame;
    frame_parser_result_t result = FrameParser_Push(&upper_parser, byte, &frame);

    if (result == FRAME_PARSER_FRAME)
    {
        uint32_t now_ms = PlatformTime_TaskNowMs();
        CommunicationCommandRouter_Handle(COMMUNICATION_LINK_UPPER, &frame, now_ms);
        upper_last_rx_timestamp_ms      = now_ms;
        upper_state.last_valid_frame_ms = now_ms;
    }
    else if (result == FRAME_PARSER_CHECKSUM_ERROR)
    {
        upper_state.rx_checksum_errors++;
    }
}

static void UpperUartService_PollRx(void)
{
    uint16_t write_pos;

    write_pos = UpperUartTransport_GetRxWritePosition(UPPER_UART_RX_BUFFER_SIZE);
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
                UpperUartService_RestartRxDma(1U);
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
        if (advanced > 0U && unread >= (UPPER_UART_RX_BUFFER_SIZE - UPPER_PROTOCOL_MAX_FRAME))
        {
            upper_state.rx_overwrite_count++;
            UpperUartService_RestartRxDma(1U);
            return;
        }
        upper_last_write_pos = write_pos;
    }
    while (upper_rx_read_pos != write_pos)
    {
        UpperUartService_ProcessByte(upper_rx_dma_buffer[upper_rx_read_pos]);
        upper_rx_read_pos++;
        if (upper_rx_read_pos >= UPPER_UART_RX_BUFFER_SIZE)
        {
            upper_rx_read_pos = 0U;
        }
    }
}

static void UpperUartService_SendStatus(uint32_t now_ms, const system_snapshot_t *snapshot)
{
    uint16_t frame_len;

    if ((now_ms - upper_last_status_ms) < UPPER_UART_STATUS_PERIOD_MS)
    {
        return;
    }

    frame_len =
        TelemetryFrameBuilder_BuildStatus(snapshot, COMMUNICATION_LINK_UPPER, upper_tx_frame, sizeof(upper_tx_frame));
    if (frame_len > 0U)
    {
        if (UpperUartService_EnqueueTx(upper_tx_frame, frame_len, UPPER_TX_PRIORITY_STATUS) != 0U)
        {
            upper_last_status_ms = now_ms;
        }
    }
}

void UpperUartService_Init(void)
{
    upper_rx_read_pos        = 0U;
    upper_last_write_pos     = 0U;
    upper_last_status_ms     = 0U;
    upper_last_imu_status_ms = 0U;
    upper_last_diagnostic_ms = 0U;
    upper_state              = (upper_uart_service_state_t){0};
    memset(upper_tx_queue, 0, sizeof(upper_tx_queue));
    upper_tx_queue_count       = 0U;
    upper_tx_active_slot       = UPPER_UART_TX_SLOT_INVALID;
    upper_tx_sequence          = 0UL;
    upper_last_rx_timestamp_ms = 0U;
    UpperUartService_ResetParser();
    UpperUartTransport_StartRx(upper_rx_dma_buffer, UPPER_UART_RX_BUFFER_SIZE);
}

static void UpperUartService_SendDiagnostic(uint32_t now_ms, const system_snapshot_t *snapshot)
{
    uint16_t frame_len;

    if ((now_ms - upper_last_diagnostic_ms) < UPPER_DIAGNOSTIC_PERIOD_MS)
    {
        return;
    }

    frame_len = TelemetryFrameBuilder_BuildDiagnostic(snapshot,
                                                      now_ms,
                                                      upper_diagnostic_tx_frame,
                                                      sizeof(upper_diagnostic_tx_frame));
    if (frame_len > 0U
        && UpperUartService_EnqueueTx(upper_diagnostic_tx_frame, frame_len, UPPER_TX_PRIORITY_DIAGNOSTIC) != 0U)
    {
        upper_last_diagnostic_ms = now_ms;
    }
}

static void UpperUartService_SendImuStatus(uint32_t now_ms, const system_snapshot_t *snapshot)
{
    uint16_t frame_len;

    if ((now_ms - upper_last_imu_status_ms) < UPPER_IMU_STATUS_PERIOD_MS)
    {
        return;
    }

    frame_len = TelemetryFrameBuilder_BuildImu(snapshot, now_ms, upper_imu_tx_frame, sizeof(upper_imu_tx_frame));
    if (frame_len > 0U)
    {
        if (UpperUartService_EnqueueTx(upper_imu_tx_frame, frame_len, UPPER_TX_PRIORITY_IMU) != 0U)
        {
            upper_last_imu_status_ms = now_ms;
        }
    }
}

void UpperUartService_Update(void)
{
    system_snapshot_t snapshot = {0};
    uint32_t          now_ms   = PlatformTime_TaskNowMs();

    UpperUartService_PollRx();
    (void)SystemSnapshotService_Get(&snapshot);
    UpperUartService_SendStatus(now_ms, &snapshot);
    UpperUartService_SendDiagnostic(now_ms, &snapshot);
    UpperUartService_SendImuStatus(now_ms, &snapshot);
}

void UpperUartService_GetState(upper_uart_service_state_t *state)
{
    if (state != 0)
    {
        *state = upper_state;
    }
}

void UpperUartService_OnUartError(void)
{
    uint32_t primask;

    upper_state.uart_errors++;
    UpperUartService_RestartRxDma(1U);
    primask = PlatformCritical_Enter();
    if (upper_tx_active_slot != UPPER_UART_TX_SLOT_INVALID && upper_tx_queue[upper_tx_active_slot].used != 0U)
    {
        upper_tx_queue[upper_tx_active_slot].used = 0U;
        upper_tx_queue_count--;
        upper_state.tx_busy_drops++;
    }
    upper_tx_active_slot = UPPER_UART_TX_SLOT_INVALID;
    UpperUartService_TryStartTxLocked();
    PlatformCritical_Exit(primask);
}

uint32_t UpperUartService_GetLastRxTimestamp(void)
{
    return upper_last_rx_timestamp_ms;
}

void UpperUartService_OnDmaHalf(void)
{
    upper_state.rx_dma_half_count++;
}

void UpperUartService_OnDmaFull(void)
{
    upper_state.rx_dma_full_count++;
}

void UpperUartService_OnTxComplete(void)
{
    uint32_t primask = PlatformCritical_Enter();
    if (upper_tx_active_slot != UPPER_UART_TX_SLOT_INVALID && upper_tx_queue[upper_tx_active_slot].used != 0U)
    {
        upper_tx_queue[upper_tx_active_slot].used = 0U;
        upper_tx_queue_count--;
        upper_state.tx_frames++;
    }
    upper_tx_active_slot = UPPER_UART_TX_SLOT_INVALID;
    UpperUartService_TryStartTxLocked();
    PlatformCritical_Exit(primask);
}
