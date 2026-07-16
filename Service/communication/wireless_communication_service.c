#include "wireless_communication_service.h"
#include "remote_command_dispatcher.h"
#include "framed_stream_parser.h"
#include "telemetry_encoder.h"
#include "platform_time.h"
#include "esp12f_transport.h"

#include "robot_link_protocol.h"

#define ESP12F_RX_RING_SIZE            128U
#define ESP12F_RX_INTERBYTE_TIMEOUT_MS 100U

static volatile uint8_t               esp12f_rx_ring[ESP12F_RX_RING_SIZE] __attribute__((aligned(4)));
static volatile uint16_t              esp12f_rx_head;
static volatile uint16_t              esp12f_rx_tail;
static uint8_t                        esp12f_rx_byte;
static frame_parser_t                 esp12f_parser;
static uint32_t                       esp12f_parser_last_byte_ms;
static uint8_t                        esp12f_tx_frame[ROBOT_LINK_PROTOCOL_MAX_FRAME];
static uint32_t                       esp12f_last_status_ms;
static uint8_t                        esp12f_diagnostic_tx_frame[ROBOT_LINK_PROTOCOL_MAX_FRAME];
static uint32_t                       esp12f_last_diagnostic_ms;
static wireless_communication_state_t esp12f_state;
static communication_config_t         esp12f_config;
static uint8_t                        esp12f_isolated;

static void WirelessCommunication_ResetParser(void)
{
    FrameParser_Reset(&esp12f_parser);
    esp12f_parser_last_byte_ms = 0U;
}

static void WirelessCommunication_ProcessByte(uint8_t byte)
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
        RemoteCommandDispatcher_Handle(COMMUNICATION_LINK_ESP12F, &frame, now_ms);
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

static void WirelessCommunication_PollRx(void)
{
    while (esp12f_rx_tail != esp12f_rx_head)
    {
        uint8_t byte   = esp12f_rx_ring[esp12f_rx_tail];
        esp12f_rx_tail = (uint16_t)((esp12f_rx_tail + 1U) % ESP12F_RX_RING_SIZE);
        WirelessCommunication_ProcessByte(byte);
    }
}

static void WirelessCommunication_SendStatus(uint32_t now_ms, const communication_publish_model_t *snapshot)
{
    uint16_t frame_len;

    if ((now_ms - esp12f_last_status_ms) < esp12f_config.esp12f_status_period_ms)
    {
        return;
    }
    esp12f_last_status_ms = now_ms;
    if (Esp12fTransport_IsTxReady() == 0U)
    {
        esp12f_state.tx_busy_drops++;
        return;
    }

    frame_len =
        TelemetryEncoder_BuildStatus(snapshot, COMMUNICATION_LINK_ESP12F, esp12f_tx_frame, sizeof(esp12f_tx_frame));
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

static void WirelessCommunication_SendDiagnostic(uint32_t now_ms, const communication_publish_model_t *snapshot)
{
    uint16_t frame_len;

    if ((now_ms - esp12f_last_diagnostic_ms) < esp12f_config.host_diagnostic_period_ms
        || Esp12fTransport_IsTxReady() == 0U)
    {
        return;
    }

    frame_len = TelemetryEncoder_BuildDiagnostic(snapshot,
                                                 now_ms,
                                                 esp12f_diagnostic_tx_frame,
                                                 sizeof(esp12f_diagnostic_tx_frame));
    if (frame_len > 0U && Esp12fTransport_TransmitAsync(esp12f_diagnostic_tx_frame, frame_len) == TRANSPORT_STATUS_OK)
    {
        esp12f_last_diagnostic_ms = now_ms;
        esp12f_state.tx_frames++;
    }
}

uint8_t WirelessCommunication_Init(const communication_config_t *config)
{
    if (config == 0 || config->esp12f_status_period_ms == 0U || config->host_diagnostic_period_ms == 0U)
    {
        return 0U;
    }
    esp12f_config                     = *config;
    esp12f_isolated                   = 0U;
    esp12f_rx_head                    = 0U;
    esp12f_rx_tail                    = 0U;
    esp12f_last_status_ms             = 0U;
    esp12f_last_diagnostic_ms         = 0U;
    esp12f_state                      = (wireless_communication_state_t){0};
    esp12f_state.last_rx_timestamp_ms = 0U;
    WirelessCommunication_SetDownloadMode(0U);
    WirelessCommunication_RestartRx();
    return 1U;
}

void WirelessCommunication_RestartRx(void)
{
    esp12f_rx_head = 0U;
    esp12f_rx_tail = 0U;
    WirelessCommunication_ResetParser();
    Esp12fTransport_StartRx(&esp12f_rx_byte);
}

void WirelessCommunication_Update(const communication_publish_model_t *publish_model)
{
    uint32_t now_ms = PlatformTime_TaskNowMs();

    if (esp12f_isolated != 0U)
    {
        return;
    }

    if (FrameParser_IsIdle(&esp12f_parser) == 0U
        && (uint32_t)(now_ms - esp12f_parser_last_byte_ms) > ESP12F_RX_INTERBYTE_TIMEOUT_MS)
    {
        esp12f_state.rx_timeout_resets++;
        WirelessCommunication_ResetParser();
    }
    WirelessCommunication_PollRx();
    if (publish_model != 0)
    {
        WirelessCommunication_SendStatus(now_ms, publish_model);
        WirelessCommunication_SendDiagnostic(now_ms, publish_model);
    }
}

void WirelessCommunication_ResetModule(void)
{
    Esp12fTransport_ResetModule();
}

void WirelessCommunication_Isolate(void)
{
    esp12f_isolated = 1U;
    Esp12fTransport_Isolate();
}

void WirelessCommunication_SetDownloadMode(uint8_t enabled)
{
    esp12f_state.boot_mode_download = (enabled != 0U) ? 1U : 0U;
    Esp12fTransport_SetDownloadMode(enabled);
}

void WirelessCommunication_OnRxCplt(void)
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

void WirelessCommunication_OnUartError(void)
{
    esp12f_state.uart_errors++;
    esp12f_rx_head = 0U;
    esp12f_rx_tail = 0U;
    WirelessCommunication_ResetParser();
    Esp12fTransport_StartRx(&esp12f_rx_byte);
}

void WirelessCommunication_GetState(wireless_communication_state_t *state)
{
    if (state != 0)
    {
        *state = esp12f_state;
    }
}
