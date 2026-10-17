#include "esp12f_flash_bridge.h"

#include "command_management_service.h"
#include "motion_maintenance_orchestrator.h"
#include "motion_control_service.h"
#include "motion_control_maintenance.h"
#include "esp12f_boot_control.h"
#include "wireless_communication_service.h"
#include "platform_time.h"
#include "uart_bridge_transport.h"
#include "usart1_debug_console.h"
#include "debug_uart_transport.h"

#define ESP12F_FLASH_BRIDGE_IDLE_TIMEOUT_MS 30000U

typedef enum
{
    ESP12F_BRIDGE_OFF = 0,
    ESP12F_BRIDGE_PREPARING,
    ESP12F_BRIDGE_STARTING,
    ESP12F_BRIDGE_FORWARDING,
    ESP12F_BRIDGE_RESTORING
} esp12f_bridge_phase_t;

static volatile esp12f_bridge_phase_t bridge_phase;
static uint8_t                        bridge_download_mode;
static uint8_t                        bridge_maintenance_lock_held;
static uint32_t                       bridge_auto_exit_count;
static uint32_t                       bridge_last_auto_exit_idle_ms;

static void Esp12fFlashBridge_ReleaseMaintenance(void)
{
    if (bridge_maintenance_lock_held != 0U)
    {
        bridge_maintenance_lock_held = 0U;
        AppMotionMaintenance_End();
    }
}

static void Esp12fFlashBridge_RestoreServices(void)
{
    Usart1DebugConsole_RestartRx();
    WirelessCommunication_RestartRx();
}

static void Esp12fFlashBridge_BeginRestore(void)
{
    bridge_phase = ESP12F_BRIDGE_RESTORING;
    UartBridgeTransport_Stop();
    bridge_download_mode = 0U;
    Esp12fBootControl_EnterNormal();
}

void Esp12fFlashBridge_Init(void)
{
    bridge_phase                  = ESP12F_BRIDGE_OFF;
    bridge_download_mode          = 0U;
    bridge_maintenance_lock_held  = 0U;
    bridge_auto_exit_count        = 0U;
    bridge_last_auto_exit_idle_ms = 0U;
    UartBridgeTransport_Init();
}

uint8_t Esp12fFlashBridge_Enable(uint8_t download_mode)
{
    if (bridge_phase == ESP12F_BRIDGE_RESTORING)
    {
        return 0U;
    }
    if (bridge_phase != ESP12F_BRIDGE_OFF)
    {
        return 1U;
    }
    if (AppMotionMaintenance_Begin() != APP_MOTION_MAINTENANCE_OK)
    {
        return 0U;
    }
    bridge_maintenance_lock_held = 1U;
    /* Reserve both console and wireless tasks before touching either UART. */
    bridge_phase = ESP12F_BRIDGE_PREPARING;

    CommandManagement_ClearAll();
    Usart1DebugConsole_RevokeMaintenanceAuthorization();
    MotionControl_CancelTestMode();
    DebugUartTransport_Suspend();
    UartBridgeTransport_Stop();
    Usart1DebugConsole_ClearRxBuffer();

    bridge_download_mode = (download_mode != 0U) ? 1U : 0U;
    if (bridge_download_mode != 0U)
    {
        Esp12fBootControl_EnterDownload();
    }
    else
    {
        Esp12fBootControl_EnterNormal();
    }

    bridge_phase = ESP12F_BRIDGE_STARTING;
    return 1U;
}

void Esp12fFlashBridge_Disable(void)
{
    if (bridge_phase == ESP12F_BRIDGE_OFF)
    {
        bridge_download_mode = 0U;
        WirelessCommunication_SetDownloadMode(0U);
        Esp12fFlashBridge_ReleaseMaintenance();
        return;
    }

    if (bridge_phase != ESP12F_BRIDGE_RESTORING)
    {
        Esp12fFlashBridge_BeginRestore();
    }
}

uint8_t Esp12fFlashBridge_IsActive(void)
{
    return (bridge_phase != ESP12F_BRIDGE_OFF) ? 1U : 0U;
}

uint32_t Esp12fFlashBridge_GetIdleMs(void)
{
    return UartBridgeTransport_GetIdleMs();
}

void Esp12fFlashBridge_Update(uint32_t now_ms)
{
    uint32_t idle_ms;

    (void)now_ms;
    if (bridge_phase == ESP12F_BRIDGE_STARTING)
    {
        if (Esp12fBootControl_Update(PlatformTime_NowMs()) != 0U)
        {
            if (UartBridgeTransport_Start(PlatformTime_NowMs()) != 0U)
            {
                bridge_phase = ESP12F_BRIDGE_FORWARDING;
            }
            else
            {
                Esp12fFlashBridge_BeginRestore();
            }
        }
        return;
    }
    if (bridge_phase == ESP12F_BRIDGE_RESTORING)
    {
        if (Esp12fBootControl_Update(PlatformTime_NowMs()) != 0U)
        {
            Esp12fFlashBridge_RestoreServices();
            bridge_phase = ESP12F_BRIDGE_OFF;
            Esp12fFlashBridge_ReleaseMaintenance();
        }
        return;
    }
    if (bridge_phase != ESP12F_BRIDGE_FORWARDING)
    {
        return;
    }

    UartBridgeTransport_Process();
    idle_ms = UartBridgeTransport_GetIdleMs();
    if (idle_ms >= ESP12F_FLASH_BRIDGE_IDLE_TIMEOUT_MS)
    {
        bridge_last_auto_exit_idle_ms = idle_ms;
        bridge_auto_exit_count++;
        Esp12fFlashBridge_Disable();
    }
}

void Esp12fFlashBridge_GetState(esp12f_flash_bridge_state_t *state)
{
    uart_bridge_transport_state_t transport;

    if (state == 0)
    {
        return;
    }
    UartBridgeTransport_GetState(&transport);
    *state = (esp12f_flash_bridge_state_t){
        .active                 = Esp12fFlashBridge_IsActive(),
        .download_mode          = bridge_download_mode,
        .pc_to_esp_rx_bytes     = transport.pc_to_esp_rx_bytes,
        .esp_to_pc_rx_bytes     = transport.esp_to_pc_rx_bytes,
        .pc_to_esp_tx_bytes     = transport.pc_to_esp_tx_bytes,
        .esp_to_pc_tx_bytes     = transport.esp_to_pc_tx_bytes,
        .pc_to_esp_overflow     = transport.pc_to_esp_overflow,
        .esp_to_pc_overflow     = transport.esp_to_pc_overflow,
        .uart_error_count       = transport.uart_error_count,
        .auto_exit_count        = bridge_auto_exit_count,
        .last_auto_exit_idle_ms = bridge_last_auto_exit_idle_ms,
        .rx_start_errors        = transport.rx_start_errors,
        .last_activity_ms       = transport.last_activity_ms,
    };
}
