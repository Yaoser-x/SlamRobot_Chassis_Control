#include "esp12f_flash_bridge.h"

#include "motion_maintenance_orchestrator.h"
#include "motion_control_service.h"
#include "motion_control_maintenance.h"
#include "uart_bridge_transport.h"
#include "app_tasks.h"
#include "robot_config.h"
#include "remote_operation_orchestrator.h"
#include "system_monitoring_service.h"
#include "system_publish_snapshot_service.h"
#include "platform_reset_trace.h"

#include <assert.h>
#include <setjmp.h>
#include <stdio.h>

static app_motion_maintenance_result_t maintenance_result;
static uart_bridge_transport_state_t   transport_state;
static uint8_t                         transport_start_ok;
static uint32_t                        transport_idle_ms;
static uint32_t                        maintenance_end_count;
static uint32_t                        normal_boot_count;
static uint32_t                        download_boot_count;
static uint32_t                        debug_restart_count;
static uint32_t                        esp_restart_count;
static uint32_t                        fake_now_ms;
static uint8_t                         debug_suspended;
static jmp_buf                         task_exit;
static unsigned                        task_cycles;
static unsigned                        normal_updates;
static unsigned                        operation_polls;
static uint8_t                         boot_ready;

uint32_t PlatformTime_TaskNowMs(void)
{
    return fake_now_ms;
}
void PlatformResetTrace_TaskHeartbeat(reset_trace_task_t task, uint32_t tick_ms)
{
    assert(task == RESET_TRACE_TASK_ESP && tick_ms == fake_now_ms);
}
const robot_config_t *RobotConfig_GetDefault(void)
{
    static const robot_config_t config = {.tasks = {[APP_TASK_ESP12F] = {.period_ms = 10U}}};
    return &config;
}
void SystemMonitoring_DelayUntil(system_monitoring_task_id_t task, uint32_t *next, uint32_t period)
{
    assert(task == SYSTEM_MONITORING_TASK_ESP && period == 10U);
    *next += period;
    fake_now_ms += period;
    if (--task_cycles == 0U)
        longjmp(task_exit, 1);
}
uint32_t SystemPublishSnapshot_Get(communication_publish_model_t *out)
{
    assert(Esp12fFlashBridge_IsActive() == 0U);
    *out = (communication_publish_model_t){0};
    return 1U;
}
void WirelessCommunication_Update(const communication_publish_model_t *snapshot)
{
    assert(snapshot != 0 && Esp12fFlashBridge_IsActive() == 0U);
    normal_updates++;
}
uint8_t WirelessCommunication_TakeOperation(uint32_t now, communication_operation_request_t *request)
{
    assert(now == fake_now_ms && request != 0 && Esp12fFlashBridge_IsActive() == 0U);
    operation_polls++;
    return 0U;
}
communication_operation_stage_t RemoteOperationOrchestrator_Dispatch(const communication_operation_request_t *request,
                                                                     uint32_t                                *detail)
{
    (void)request;
    (void)detail;
    assert(0 && "No operation was offered");
    return (communication_operation_stage_t)0U;
}
void WirelessCommunication_CompleteOperation(const communication_operation_request_t *request,
                                             communication_operation_stage_t          stage,
                                             uint32_t                                 detail,
                                             uint32_t                                 now)
{
    (void)request;
    (void)stage;
    (void)detail;
    (void)now;
    assert(0 && "No operation was dispatched");
}

void DebugUartTransport_Suspend(void)
{
    assert(transport_state.active == 0U);
    debug_suspended = 1U;
}

app_motion_maintenance_result_t AppMotionMaintenance_Begin(void)
{
    return maintenance_result;
}
void AppMotionMaintenance_End(void)
{
    maintenance_end_count++;
}
void CommandManagement_ClearAll(void)
{
}
void Usart1DebugConsole_RevokeMaintenanceAuthorization(void)
{
}
void MotionControl_CancelTestMode(void)
{
}
void Usart1DebugConsole_ClearRxBuffer(void)
{
}
void Usart1DebugConsole_RestartRx(void)
{
    debug_suspended = 0U;
    debug_restart_count++;
}
void WirelessCommunication_RestartRx(void)
{
    esp_restart_count++;
}
void WirelessCommunication_SetDownloadMode(uint8_t enabled)
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
uint8_t Esp12fBootControl_Update(uint32_t now_ms)
{
    assert(now_ms == fake_now_ms);
    return boot_ready;
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
    assert(debug_suspended != 0U);
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
    maintenance_result    = APP_MOTION_MAINTENANCE_OK;
    transport_start_ok    = 1U;
    transport_idle_ms     = 0U;
    maintenance_end_count = 0U;
    normal_boot_count     = 0U;
    download_boot_count   = 0U;
    debug_restart_count   = 0U;
    esp_restart_count     = 0U;
    fake_now_ms           = 1000U;
    debug_suspended       = 0U;
    normal_updates        = 0U;
    operation_polls       = 0U;
    boot_ready            = 1U;
    Esp12fFlashBridge_Init();
}

static void RunThreeTaskCycles(void)
{
    task_cycles = 3U;
    if (setjmp(task_exit) == 0)
        Task_Esp12f(0);
}

static void TestTaskRespectsBridgeOwnershipAndResumes(void)
{
    ResetFakes();
    boot_ready = 0U;
    assert(Esp12fFlashBridge_Enable(1U) != 0U);
    RunThreeTaskCycles();
    assert(normal_updates == 0U && operation_polls == 0U);
    assert(transport_state.active == 0U && debug_suspended != 0U);
    boot_ready = 1U;
    RunThreeTaskCycles();
    assert(Esp12fFlashBridge_IsActive() != 0U);
    assert(transport_state.active != 0U);
    transport_idle_ms = 30000U;
    boot_ready        = 0U;
    RunThreeTaskCycles();
    assert(Esp12fFlashBridge_IsActive() != 0U && transport_state.active == 0U);
    assert(normal_updates == 0U && operation_polls == 0U && maintenance_end_count == 0U);
    assert(Esp12fFlashBridge_Enable(1U) == 0U); /* Restoration cannot be preempted. */
    boot_ready = 1U;
    RunThreeTaskCycles();
    assert(Esp12fFlashBridge_IsActive() == 0U && debug_suspended == 0U);
    assert(normal_updates == 3U && operation_polls == 3U);
    assert(maintenance_end_count == 1U && normal_boot_count == 1U);
}

static void TestDownloadAndAutoExit(void)
{
    esp12f_flash_bridge_state_t state;

    ResetFakes();
    assert(Esp12fFlashBridge_Enable(1U) != 0U);
    assert(download_boot_count == 1U);
    assert(Esp12fFlashBridge_IsActive() != 0U);
    Esp12fFlashBridge_Update(1000U);
    assert(transport_state.active != 0U);
    transport_idle_ms = 30000U;
    Esp12fFlashBridge_Update(31000U);
    assert(Esp12fFlashBridge_IsActive() != 0U && maintenance_end_count == 0U);
    Esp12fFlashBridge_Update(31001U);
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
    assert(Esp12fFlashBridge_Enable(1U) != 0U); /* Accepted; RX startup happens asynchronously. */
    Esp12fFlashBridge_Update(fake_now_ms);
    assert(download_boot_count == 1U && normal_boot_count == 1U);
    assert(debug_restart_count == 0U && maintenance_end_count == 0U);
    Esp12fFlashBridge_Update(fake_now_ms);
    assert(debug_restart_count == 1U && esp_restart_count == 1U);
    assert(maintenance_end_count == 1U);

    ResetFakes();
    maintenance_result = APP_MOTION_MAINTENANCE_BUSY;
    assert(Esp12fFlashBridge_Enable(0U) == 0U);
    assert(normal_boot_count == 0U && maintenance_end_count == 0U);

    ResetFakes();
    boot_ready = 0U;
    assert(Esp12fFlashBridge_Enable(1U) != 0U);
    Esp12fFlashBridge_Disable();
    assert(Esp12fFlashBridge_IsActive() != 0U && normal_boot_count == 1U);
    assert(maintenance_end_count == 0U);
    boot_ready = 1U;
    Esp12fFlashBridge_Update(fake_now_ms);
    assert(Esp12fFlashBridge_IsActive() == 0U && maintenance_end_count == 1U);
}

int main(void)
{
    TestDownloadAndAutoExit();
    TestStartFailureAndMaintenanceRejection();
    TestTaskRespectsBridgeOwnershipAndResumes();
    puts("PASS: ESP12F flash bridge policy");
    return 0;
}
