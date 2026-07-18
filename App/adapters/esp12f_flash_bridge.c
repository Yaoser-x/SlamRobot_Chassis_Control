#include "esp12f_flash_bridge.h"

#include "command_management_service.h"
#include "motion_control_service.h"
#include "motion_control_maintenance.h"
#include "esp12f_boot_control.h"
#include "wireless_communication_service.h"
#include "platform_time.h"
#include "uart_bridge_transport.h"
#include "usart1_debug_console.h"

#define ESP12F_FLASH_BRIDGE_IDLE_TIMEOUT_MS 30000U

static uint8_t  bridge_download_mode;
static uint8_t  bridge_maintenance_lock_held;
static uint32_t bridge_auto_exit_count;
static uint32_t bridge_last_auto_exit_idle_ms;

static void Esp12fFlashBridge_ReleaseMaintenance(void)
{
    if (bridge_maintenance_lock_held != 0U)
    {
        bridge_maintenance_lock_held = 0U;
        MotionControl_EndMaintenance();
    }
}

static void Esp12fFlashBridge_RestoreServices(void)
{
    Esp12fBootControl_EnterNormal();
    Usart1DebugConsole_RestartRx();
    WirelessCommunication_RestartRx();
}

void Esp12fFlashBridge_Init(void)
{
    bridge_download_mode          = 0U;
    bridge_maintenance_lock_held  = 0U;
    bridge_auto_exit_count        = 0U;
    bridge_last_auto_exit_idle_ms = 0U;
    UartBridgeTransport_Init();
}

uint8_t Esp12fFlashBridge_Enable(uint8_t download_mode)
{
    uint32_t initial_activity_ms;

    if (UartBridgeTransport_IsActive() != 0U)
    {
        return 1U;
    }
    if (MotionControl_BeginMaintenance() != MOTION_CONTROL_MAINTENANCE_OK)
    {
        return 0U;
    }
    bridge_maintenance_lock_held = 1U;

    CommandManagement_ClearAll();
    Usart1DebugConsole_RevokeMaintenanceAuthorization();
    MotionControl_CancelTestMode();
    UartBridgeTransport_Stop();
    Usart1DebugConsole_ClearRxBuffer();

    initial_activity_ms  = PlatformTime_NowMs();
    bridge_download_mode = (download_mode != 0U) ? 1U : 0U;
    if (bridge_download_mode != 0U)
    {
        Esp12fBootControl_EnterDownload();
    }
    else
    {
        Esp12fBootControl_EnterNormal();
    }

    if (UartBridgeTransport_Start(initial_activity_ms) == 0U)
    {
        bridge_download_mode = 0U;
        Esp12fFlashBridge_RestoreServices();
        Esp12fFlashBridge_ReleaseMaintenance();
        return 0U;
    }
    return 1U;
}

void Esp12fFlashBridge_Disable(void)
{
    if (UartBridgeTransport_IsActive() == 0U)
    {
        bridge_download_mode = 0U;
        WirelessCommunication_SetDownloadMode(0U);
        Esp12fFlashBridge_ReleaseMaintenance();
        return;
    }

    UartBridgeTransport_Stop();
    bridge_download_mode = 0U;
    Esp12fFlashBridge_RestoreServices();
    Esp12fFlashBridge_ReleaseMaintenance();
}

uint8_t Esp12fFlashBridge_IsActive(void)
{
    return UartBridgeTransport_IsActive();
}

uint32_t Esp12fFlashBridge_GetIdleMs(void)
{
    return UartBridgeTransport_GetIdleMs();
}

void Esp12fFlashBridge_Update(uint32_t now_ms)
{
    uint32_t idle_ms;

    (void)now_ms;
    if (UartBridgeTransport_IsActive() == 0U)
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
        .active                 = transport.active,
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
