#include "usart1_debug_console.h"

#include "motion_control_service.h"
#include "cmsis_os2.h"
#include "command_management_service.h"
#include "debug_maintenance_policy.h"
#include "debug_telemetry.h"
#include "debug_console_writer.h"
#include "debug_uart_transport.h"
#include "debug_cmd_control.h"
#include "debug_cmd_current.h"
#include "debug_cmd_esp12f.h"
#include "debug_cmd_imu.h"
#include "debug_cmd_line.h"
#include "debug_cmd_param.h"
#include "debug_cmd_system.h"
#include "debug_system_status.h"
#include "esp12f_flash_bridge.h"
#include "platform_reset_trace.h"
#include "safety_management_service.h"

#include <stdio.h>
#include <string.h>

#define DEBUG_CONSOLE_RX_LINE_SIZE   96U
#define DEBUG_CONSOLE_TX_LINE_SIZE   1536U
#define DEBUG_CONSOLE_TASK_PERIOD_MS 10U
#define DEBUG_CONSOLE_LOG_PERIOD_MS  500U

#ifndef DEBUG_CONSOLE_RELEASE_REQUIRES_ARM
#define DEBUG_CONSOLE_RELEASE_REQUIRES_ARM 0U
#endif

/* ────────── 日志级别宏 ────────── */
#define LOG_INFO(fmt, ...)                                                                                             \
    do                                                                                                                 \
    {                                                                                                                  \
        char _log_tx[DEBUG_CONSOLE_TX_LINE_SIZE];                                                                      \
        (void)snprintf(_log_tx, sizeof(_log_tx), "[INFO] " fmt "\r\n", ##__VA_ARGS__);                                 \
        DebugConsole_Write(_log_tx);                                                                                   \
    } while (0)

#define LOG_WARN(fmt, ...)                                                                                             \
    do                                                                                                                 \
    {                                                                                                                  \
        char _log_tx[DEBUG_CONSOLE_TX_LINE_SIZE];                                                                      \
        (void)snprintf(_log_tx, sizeof(_log_tx), "[WARN] " fmt "\r\n", ##__VA_ARGS__);                                 \
        DebugConsole_Write(_log_tx);                                                                                   \
    } while (0)

#define LOG_ERR(fmt, ...)                                                                                              \
    do                                                                                                                 \
    {                                                                                                                  \
        char _log_tx[DEBUG_CONSOLE_TX_LINE_SIZE];                                                                      \
        (void)snprintf(_log_tx, sizeof(_log_tx), "[ERR] " fmt "\r\n", ##__VA_ARGS__);                                  \
        DebugConsole_Write(_log_tx);                                                                                   \
    } while (0)

#define DebugConsole_Write DebugConsoleWriter_Write

static char                       rx_line[DEBUG_CONSOLE_RX_LINE_SIZE];
static uint8_t                    rx_len;
static uint8_t                    debug_velocity_enabled;
static uint32_t                   debug_velocity_generation;
static command_velocity_t         debug_velocity_cmd;
static debug_maintenance_policy_t maintenance_policy;

static uint8_t DebugConsole_MotorTestAllowed(void)
{
    if (SafetyManagement_IsEmergencyStop() != 0U || SafetyManagement_IsFaultStop() != 0U)
    {
        Usart1DebugConsole_RevokeMaintenanceAuthorization();
        return 0U;
    }
    return DebugMaintenancePolicy_Allowed(&maintenance_policy,
                                          osKernelGetTickCount(),
                                          DEBUG_CONSOLE_RELEASE_REQUIRES_ARM);
}

static void DebugConsole_HandleLine(char *line)
{
    if (DebugCmdSystem_TryHandle(line) != 0U)
    {
        return;
    }
    if (DebugCmdParam_TryHandle(line) != 0U)
    {
        return;
    }
    if (DebugCmdImu_TryHandle(line) != 0U)
    {
        return;
    }
    if (DebugCmdEsp12f_TryHandle(line, &debug_velocity_enabled) != 0U)
    {
        return;
    }
    {
        const debug_cmd_control_context_t context = {
            .velocity_enabled    = &debug_velocity_enabled,
            .velocity_generation = &debug_velocity_generation,
            .velocity_command    = &debug_velocity_cmd,
            .motor_test_allowed  = DebugConsole_MotorTestAllowed,
            .revoke_maintenance  = Usart1DebugConsole_RevokeMaintenanceAuthorization,
        };
        if (DebugCmdControl_TryHandle(line, &context) != 0U)
        {
            return;
        }
    }

    if (strcmp(line, "maint arm") == 0)
    {
        DebugMaintenancePolicy_Arm(&maintenance_policy, osKernelGetTickCount());
        LOG_INFO("maintenance authorization armed for 60s");
    }
    else if (strcmp(line, "maint off") == 0)
    {
        Usart1DebugConsole_RevokeMaintenanceAuthorization();
        LOG_INFO("maintenance authorization revoked");
    }
    else if (DebugCmdCurrent_TryHandle(line) != 0U)
    {
    }
    else if (DebugTelemetry_TryHandle(line) != 0U)
    {
    }
    else if (DebugCmdLine_TryHandle(line) != 0U)
    {
    }
    else
    {
        LOG_ERR("unknown command, type help");
    }
}

static void DebugConsole_PollRx(void)
{
    uint8_t ch;

    while (DebugUartTransport_ReadByte(&ch) != 0U)
    {
        if ((ch == '\r') || (ch == '\n'))
        {
            if (rx_len > 0U)
            {
                rx_line[rx_len] = '\0';
                DebugConsole_HandleLine(rx_line);
                rx_len = 0U;
                if (Esp12fFlashBridge_IsActive() != 0U)
                {
                    Usart1DebugConsole_ClearRxBuffer();
                    break;
                }
            }
        }
        else if (rx_len < (DEBUG_CONSOLE_RX_LINE_SIZE - 1U))
        {
            rx_line[rx_len++] = (char)ch;
        }
        else
        {
            rx_len = 0U;
            LOG_WARN("line too long");
        }
    }
}

void Usart1DebugConsole_Init(void)
{
    rx_len                    = 0U;
    debug_velocity_enabled    = 0U;
    debug_velocity_generation = CommandManagement_GetMotionRevokeGeneration();
    DebugMaintenancePolicy_Init(&maintenance_policy);
    DebugTelemetry_Init();
    debug_velocity_cmd = (command_velocity_t){0};
    DebugUartTransport_Init();
    DebugConsole_Write("\r\nF407 V2 chassis firmware\r\n");
    DebugSystemStatus_PrintResetFlags();
    DebugSystemStatus_PrintResetTrace();
    DebugConsole_Write("USART1 debug console ready, type help\r\n");
}

void Usart1DebugConsole_ClearRxBuffer(void)
{
    rx_len = 0U;
    DebugUartTransport_ClearRx();
}

void Usart1DebugConsole_RestartRx(void)
{
    rx_len = 0U;
    DebugUartTransport_RestartRx();
}

void Usart1DebugConsole_OnRxCplt(void)
{
    if (DebugUartTransport_OnRxComplete() != 0U)
    {
        Usart1DebugConsole_RevokeMaintenanceAuthorization();
    }
}

void Usart1DebugConsole_OnUartError(void)
{
    Usart1DebugConsole_RevokeMaintenanceAuthorization();
    DebugUartTransport_OnUartError();
}

void Usart1DebugConsole_RevokeMaintenanceAuthorization(void)
{
    DebugMaintenancePolicy_Revoke(&maintenance_policy);
    debug_velocity_enabled = 0U;
    MotionControl_CancelTestMode();
    CommandManagement_ClearSource(COMMAND_SOURCE_DEBUG);
}

void DebugConsole_RunTask(void *argument)
{
    (void)argument;
    for (;;)
    {
        uint32_t now_ms = osKernelGetTickCount();

        PlatformResetTrace_TaskHeartbeat(RESET_TRACE_TASK_DEBUG, now_ms);
        if (SafetyManagement_IsEmergencyStop() != 0U || SafetyManagement_IsFaultStop() != 0U)
        {
            Usart1DebugConsole_RevokeMaintenanceAuthorization();
        }
        else if (DebugMaintenancePolicy_Allowed(&maintenance_policy, now_ms, DEBUG_CONSOLE_RELEASE_REQUIRES_ARM) == 0U)
        {
            if (DEBUG_CONSOLE_RELEASE_REQUIRES_ARM != 0U)
            {
                MotionControl_CancelTestMode();
            }
        }
        if (Esp12fFlashBridge_IsActive() != 0U)
        {
            osDelay(DEBUG_CONSOLE_TASK_PERIOD_MS);
            continue;
        }

        DebugConsole_PollRx();

        if (debug_velocity_enabled != 0U && debug_velocity_generation != CommandManagement_GetMotionRevokeGeneration())
        {
            Usart1DebugConsole_RevokeMaintenanceAuthorization();
            LOG_WARN("velocity command requires a new local command");
        }
        else if (debug_velocity_enabled != 0U)
        {
            debug_velocity_cmd.timestamp_ms = now_ms;
            if (CommandManagement_SetForGeneration(&debug_velocity_cmd, debug_velocity_generation)
                != COMMAND_RESULT_ACCEPTED)
            {
                debug_velocity_enabled = 0U;
                LOG_WARN("velocity command stopped");
            }
        }

        DebugTelemetry_Step(now_ms);

        osDelay(DEBUG_CONSOLE_TASK_PERIOD_MS);
    }
}
