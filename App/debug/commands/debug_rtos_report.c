#include "debug_rtos_report.h"

#include "debug_console_writer.h"
#include "debug_uart_transport.h"
#include "esp12f_service.h"
#include "task_health_service.h"
#include "upper_uart_service.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"

#include <stdio.h>

#define DEBUG_RTOS_REPORT_TX_SIZE 1536U

extern osThreadId_t safetyTaskHandle;
extern osThreadId_t motorTaskHandle;
extern osThreadId_t rpiCommTaskHandle;
extern osThreadId_t imuTaskHandle;
extern osThreadId_t lineTaskHandle;
extern osThreadId_t espTaskHandle;
extern osThreadId_t usart1DebugTaskHandle;
extern osThreadId_t ps2TaskHandle;
extern osThreadId_t oledTaskHandle;
extern osThreadId_t ledTaskHandle;

static void
DebugConsole_PrintTaskStatus(const char *name, osThreadId_t handle, uint32_t missed, task_health_service_id_t task)
{
    char                  tx[DEBUG_RTOS_REPORT_TX_SIZE];
    chassis_task_health_t health         = {0};
    uint32_t              last_heartbeat = 0U;
    uint32_t              timeout_count  = 0U;
    uint8_t               timed_out      = 0U;

    if ((uint32_t)task < (uint32_t)TASK_HEALTH_SERVICE_COUNT)
    {
        TaskHealthService_GetHealth(&health);
        last_heartbeat = health.last_heartbeat_ms[task];
        timeout_count  = health.timeout_count[task];
        timed_out      = health.timed_out[task];
    }

    if (handle == NULL)
    {
        (void)snprintf(tx, sizeof(tx), "RTOS %-10s missing\r\n", name);
    }
    else
    {
        (void)snprintf(tx,
                       sizeof(tx),
                       "RTOS %-10s state=%ld stack_free=%luB missed=%lu hb=%lu timeout=%lu to=%u\r\n",
                       name,
                       (long)osThreadGetState(handle),
                       (unsigned long)osThreadGetStackSpace(handle),
                       (unsigned long)missed,
                       (unsigned long)last_heartbeat,
                       (unsigned long)timeout_count,
                       timed_out);
    }
    DebugConsoleWriter_Write(tx);
}

void DebugRtosReport_Print(void)
{
    char                       tx[DEBUG_RTOS_REPORT_TX_SIZE];
    upper_uart_service_state_t upper_state;
    esp12f_service_state_t     esp_state;
    uint32_t                   heap_free = (uint32_t)xPortGetFreeHeapSize();
    uint32_t                   heap_min  = (uint32_t)xPortGetMinimumEverFreeHeapSize();

    UpperUartService_GetState(&upper_state);
    Esp12fService_GetState(&esp_state);

    (void)snprintf(tx,
                   sizeof(tx),
                   "RTOS heap_free=%luB heap_min=%luB heap_used=%luB tick=%lu\r\n",
                   (unsigned long)heap_free,
                   (unsigned long)heap_min,
                   (unsigned long)((uint32_t)configTOTAL_HEAP_SIZE - heap_free),
                   (unsigned long)osKernelGetTickCount());
    DebugConsoleWriter_Write(tx);

    DebugConsole_PrintTaskStatus("safety",
                                 safetyTaskHandle,
                                 TaskHealthService_GetMissedCount(TASK_HEALTH_SERVICE_SAFETY),
                                 TASK_HEALTH_SERVICE_SAFETY);
    DebugConsole_PrintTaskStatus("motor",
                                 motorTaskHandle,
                                 TaskHealthService_GetMissedCount(TASK_HEALTH_SERVICE_MOTOR),
                                 TASK_HEALTH_SERVICE_MOTOR);
    DebugConsole_PrintTaskStatus("rpi",
                                 rpiCommTaskHandle,
                                 TaskHealthService_GetMissedCount(TASK_HEALTH_SERVICE_RPI),
                                 TASK_HEALTH_SERVICE_RPI);
    DebugConsole_PrintTaskStatus("imu",
                                 imuTaskHandle,
                                 TaskHealthService_GetMissedCount(TASK_HEALTH_SERVICE_IMU),
                                 TASK_HEALTH_SERVICE_IMU);
    DebugConsole_PrintTaskStatus("line",
                                 lineTaskHandle,
                                 TaskHealthService_GetMissedCount(TASK_HEALTH_SERVICE_LINE),
                                 TASK_HEALTH_SERVICE_LINE);
    DebugConsole_PrintTaskStatus("esp",
                                 espTaskHandle,
                                 TaskHealthService_GetMissedCount(TASK_HEALTH_SERVICE_ESP),
                                 TASK_HEALTH_SERVICE_ESP);
    DebugConsole_PrintTaskStatus("debug", usart1DebugTaskHandle, 0U, TASK_HEALTH_SERVICE_COUNT);
    DebugConsole_PrintTaskStatus("ps2",
                                 ps2TaskHandle,
                                 TaskHealthService_GetMissedCount(TASK_HEALTH_SERVICE_PS2),
                                 TASK_HEALTH_SERVICE_PS2);
    DebugConsole_PrintTaskStatus("led",
                                 ledTaskHandle,
                                 TaskHealthService_GetMissedCount(TASK_HEALTH_SERVICE_LED),
                                 TASK_HEALTH_SERVICE_LED);
    DebugConsole_PrintTaskStatus("oled",
                                 oledTaskHandle,
                                 TaskHealthService_GetMissedCount(TASK_HEALTH_SERVICE_OLED),
                                 TASK_HEALTH_SERVICE_OLED);

    (void)snprintf(tx,
                   sizeof(tx),
                   "RTOS comm upper_tx=%lu upper_drop=%lu esp_tx=%lu esp_drop=%lu dbg_rx_ovf=%lu\r\n",
                   (unsigned long)upper_state.tx_frames,
                   (unsigned long)upper_state.tx_busy_drops,
                   (unsigned long)esp_state.tx_frames,
                   (unsigned long)esp_state.tx_busy_drops,
                   (unsigned long)DebugUartTransport_GetOverflowCount());
    DebugConsoleWriter_Write(tx);
}
