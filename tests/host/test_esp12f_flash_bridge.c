#include "esp12f_flash_bridge.h"

#include "chassis_maintenance_service.h"
#include "uart_bridge_transport.h"

#include <assert.h>
#include <stdio.h>

static chassis_maintenance_service_result_t maintenance_result;
static uart_bridge_transport_state_t        transport_state;
static uint8_t                              transport_start_ok;
static uint32_t                             transport_idle_ms;
static uint32_t                             maintenance_end_count;
static uint32_t                             normal_boot_count;
static uint32_t                             download_boot_count;
static uint32_t                             debug_restart_count;
static uint32_t                             esp_restart_count;
static uint32_t                             fake_now_ms;

chassis_maintenance_service_result_t ChassisMaintenanceService_Begin(void)
{
    return maintenance_result;
}
void ChassisMaintenanceService_End(void)
{
    maintenance_end_count++;
}
void ControlService_ClearCommand(void)
{
}
void Usart1DebugConsole_RevokeMaintenanceAuthorization(void)
{
}
void ChassisService_CancelTestMode(void)
{
}
void Usart1DebugConsole_ClearRxBuffer(void)
{
}
void Usart1DebugConsole_RestartRx(void)
{
    debug_restart_count++;
}
void Esp12fService_RestartRx(void)
{
    esp_restart_count++;
}
void Esp12fService_SetDownloadMode(uint8_t enabled)
{
    (void)enabled;
}
void Esp12fBootControl_EnterDownload(void)
{
    download_boot_count++;
}
void Esp12fBootControl_EnterNormal(void)
{
    normal_boot_count++;
}
uint32_t PlatformTime_NowMs(void)
{
    return fake_now_ms;
}
void UartBridgeTransport_Init(void)
{
    transport_state = (uart_bridge_transport_state_t){0};
}
uint8_t UartBridgeTransport_Start(uint32_t initial_activity_ms)
{
    transport_state.active           = transport_start_ok;
    transport_state.last_activity_ms = initial_activity_ms;
    return transport_start_ok;
}
void UartBridgeTransport_Stop(void)
{
    transport_state.active = 0U;
}
void UartBridgeTransport_Process(void)
{
}
uint8_t UartBridgeTransport_IsActive(void)
{
    return transport_state.active;
}
uint32_t UartBridgeTransport_GetIdleMs(void)
{
    return transport_idle_ms;
}
void UartBridgeTransport_GetState(uart_bridge_transport_state_t *state)
{
    *state = transport_state;
}

static void ResetFakes(void)
{
    maintenance_result    = CHASSIS_MAINTENANCE_SERVICE_OK;
    transport_start_ok    = 1U;
    transport_idle_ms     = 0U;
    maintenance_end_count = 0U;
    normal_boot_count     = 0U;
    download_boot_count   = 0U;
    debug_restart_count   = 0U;
    esp_restart_count     = 0U;
    fake_now_ms           = 1000U;
    Esp12fFlashBridge_Init();
}

static void TestDownloadAndAutoExit(void)
{
    esp12f_flash_bridge_state_t state;

    ResetFakes();
    assert(Esp12fFlashBridge_Enable(1U) != 0U);
    assert(download_boot_count == 1U);
    assert(Esp12fFlashBridge_IsActive() != 0U);

    transport_idle_ms = 30000U;
    Esp12fFlashBridge_Update(31000U);
    assert(Esp12fFlashBridge_IsActive() == 0U);
    assert(normal_boot_count == 1U);
    assert(debug_restart_count == 1U && esp_restart_count == 1U);
    assert(maintenance_end_count == 1U);
    Esp12fFlashBridge_GetState(&state);
    assert(state.auto_exit_count == 1U && state.last_auto_exit_idle_ms == 30000U);
}

static void TestStartFailureAndMaintenanceRejection(void)
{
    ResetFakes();
    transport_start_ok = 0U;
    assert(Esp12fFlashBridge_Enable(1U) == 0U);
    assert(download_boot_count == 1U && normal_boot_count == 1U);
    assert(debug_restart_count == 1U && esp_restart_count == 1U);
    assert(maintenance_end_count == 1U);

    ResetFakes();
    maintenance_result = CHASSIS_MAINTENANCE_SERVICE_BUSY;
    assert(Esp12fFlashBridge_Enable(0U) == 0U);
    assert(normal_boot_count == 0U && maintenance_end_count == 0U);
}

int main(void)
{
    TestDownloadAndAutoExit();
    TestStartFailureAndMaintenanceRejection();
    puts("PASS: ESP12F flash bridge policy");
    return 0;
}
