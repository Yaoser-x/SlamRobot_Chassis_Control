#include "esp12f_service.h"
#include "communication_command_router.h"
#include "framed_stream_parser.h"
#include "system_snapshot_service.h"
#include "telemetry_frame_builder.h"
#include "platform_time.h"
#include "esp12f_transport.h"

#include "control_config.h"
#include "upper_protocol.h"

#define ESP12F_RX_RING_SIZE            128U
#define ESP12F_RX_INTERBYTE_TIMEOUT_MS 100U

static volatile uint8_t       esp12f_rx_ring[ESP12F_RX_RING_SIZE] __attribute__((aligned(4)));
static volatile uint16_t      esp12f_rx_head;
static volatile uint16_t      esp12f_rx_tail;
static uint8_t                esp12f_rx_byte;
static frame_parser_t         esp12f_parser;
static uint32_t               esp12f_parser_last_byte_ms;
static uint8_t                esp12f_tx_frame[UPPER_PROTOCOL_MAX_FRAME];
static uint32_t               esp12f_last_status_ms;
static uint8_t                esp12f_diagnostic_tx_frame[UPPER_PROTOCOL_MAX_FRAME];
static uint32_t               esp12f_last_diagnostic_ms;
static esp12f_service_state_t esp12f_state;
static uint8_t                esp12f_isolated;

static void Esp12fService_ResetParser(void)
{
    FrameParser_Reset(&esp12f_parser);
    esp12f_parser_last_byte_ms = 0U;
}

static void Esp12fService_ProcessByte(uint8_t byte)
{
    protocol_frame_t      frame;
    frame_parser_result_t result;

    esp12f_parser_last_byte_ms = PlatformTime_TaskNowMs();
    result                     = FrameParser_Push(&esp12f_parser, byte, &frame);
    if (result == FRAME_PARSER_FRAME)
    {
        uint32_t now_ms = PlatformTime_TaskNowMs();
        esp12f_state.rx_frames++;
        esp12f_state.last_rx_timestamp_ms = now_ms;
        CommunicationCommandRouter_Handle(COMMUNICATION_LINK_ESP12F, &frame, now_ms);
    }
    else if (result == FRAME_PARSER_LENGTH_ERROR)
    {
        esp12f_state.rx_length_errors++;
    }
    else if (result == FRAME_PARSER_CHECKSUM_ERROR)
    {
        esp12f_state.rx_checksum_errors++;
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

static void Esp12fService_SendStatus(uint32_t now_ms, const system_snapshot_t *snapshot)
{
    uint16_t frame_len;

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

    frame_len = TelemetryFrameBuilder_BuildStatus(snapshot,
                                                  COMMUNICATION_LINK_ESP12F,
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

static void Esp12fService_SendDiagnostic(uint32_t now_ms, const system_snapshot_t *snapshot)
{
    uint16_t frame_len;

    if ((now_ms - esp12f_last_diagnostic_ms) < UPPER_DIAGNOSTIC_PERIOD_MS || Esp12fTransport_IsTxReady() == 0U)
    {
        return;
    }

    frame_len = TelemetryFrameBuilder_BuildDiagnostic(snapshot,
                                                      now_ms,
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
    system_snapshot_t snapshot = {0};
    uint32_t          now_ms   = PlatformTime_TaskNowMs();

    if (esp12f_isolated != 0U)
    {
        return;
    }

    if (FrameParser_IsIdle(&esp12f_parser) == 0U
        && (uint32_t)(now_ms - esp12f_parser_last_byte_ms) > ESP12F_RX_INTERBYTE_TIMEOUT_MS)
    {
        esp12f_state.rx_timeout_resets++;
        Esp12fService_ResetParser();
    }
    Esp12fService_PollRx();
    (void)SystemSnapshotService_Get(&snapshot);
    Esp12fService_SendStatus(now_ms, &snapshot);
    Esp12fService_SendDiagnostic(now_ms, &snapshot);
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
