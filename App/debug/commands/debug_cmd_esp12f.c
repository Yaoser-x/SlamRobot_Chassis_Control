#include "debug_cmd_esp12f.h"

#include "debug_console_writer.h"
#include "debug_telemetry.h"
#include "esp12f_flash_bridge.h"
#include "esp12f_service.h"

#include <stdio.h>
#include <string.h>

#define DEBUG_CMD_ESP_TX_SIZE 512U

#define ESP_LOG(level, fmt, ...)                                                                                       \
    do                                                                                                                 \
    {                                                                                                                  \
        char tx[DEBUG_CMD_ESP_TX_SIZE];                                                                                \
        (void)snprintf(tx, sizeof(tx), "[" level "] " fmt "\r\n", ##__VA_ARGS__);                                      \
        DebugConsoleWriter_Write(tx);                                                                                  \
    } while (0)

static void DebugCmdEsp12f_PrintStatus(void)
{
    char                        tx[DEBUG_CMD_ESP_TX_SIZE];
    esp12f_flash_bridge_state_t state;
    uint32_t                    idle_ms;

    Esp12fFlashBridge_GetState(&state);
    idle_ms = Esp12fFlashBridge_GetIdleMs();
    (void)snprintf(tx,
                   sizeof(tx),
                   "ESPFLASH active=%u download=%u idle=%lums pc_rx=%lu pc_tx=%lu esp_rx=%lu esp_tx=%lu ovf=%lu/%lu "
                   "uart_err=%lu rx_start_err=%lu auto_exit=%lu exit_idle=%lums\r\n",
                   state.active,
                   state.download_mode,
                   (unsigned long)idle_ms,
                   (unsigned long)state.pc_to_esp_rx_bytes,
                   (unsigned long)state.pc_to_esp_tx_bytes,
                   (unsigned long)state.esp_to_pc_rx_bytes,
                   (unsigned long)state.esp_to_pc_tx_bytes,
                   (unsigned long)state.pc_to_esp_overflow,
                   (unsigned long)state.esp_to_pc_overflow,
                   (unsigned long)state.uart_error_count,
                   (unsigned long)state.rx_start_errors,
                   (unsigned long)state.auto_exit_count,
                   (unsigned long)state.last_auto_exit_idle_ms);
    DebugConsoleWriter_Write(tx);
}

uint8_t DebugCmdEsp12f_TryHandle(const char *line, uint8_t *debug_velocity_enabled)
{
    int value;

    if (line == 0 || debug_velocity_enabled == 0)
    {
        return 0U;
    }
    if (strcmp(line, "espreset") == 0)
    {
        Esp12fService_ResetModule();
        ESP_LOG("INFO", "esp12f reset");
    }
    else if (strcmp(line, "espisolate") == 0)
    {
        Esp12fService_Isolate();
        ESP_LOG("INFO", "esp12f isolated until board reset");
    }
    else if (sscanf(line, "espboot %d", &value) == 1)
    {
        Esp12fService_SetDownloadMode((value != 0) ? 1U : 0U);
        ESP_LOG("INFO", "esp12f %s", (value != 0) ? "download mode" : "normal boot mode");
    }
    else if (strcmp(line, "espflash on") == 0 || strcmp(line, "espat on") == 0)
    {
        uint8_t download = (strcmp(line, "espflash on") == 0) ? 1U : 0U;
        DebugTelemetry_Stop();
        *debug_velocity_enabled = 0U;
        if (Esp12fFlashBridge_Enable(download) != 0U)
        {
            if (download != 0U)
            {
                ESP_LOG("INFO", "esp12f flash bridge on: close this terminal and use esptool/Arduino at 115200");
            }
            else
            {
                ESP_LOG("INFO",
                        "AT passthrough active: IO0=high, USART1<->USART2 bridge open.\r\n"
                        "Type AT commands directly. Auto-exit after 30s idle.");
            }
        }
        else
        {
            ESP_LOG("ERR", "%s failed: UART RX not ready", download != 0U ? "esp12f flash bridge" : "AT passthrough");
        }
    }
    else if (strcmp(line, "espflash off") == 0 || strcmp(line, "espat off") == 0)
    {
        uint8_t flash = (strcmp(line, "espflash off") == 0) ? 1U : 0U;
        Esp12fFlashBridge_Disable();
        ESP_LOG("INFO", "%s off, normal boot requested", flash != 0U ? "esp12f flash bridge" : "AT passthrough");
    }
    else if (strcmp(line, "espflash status") == 0)
    {
        DebugCmdEsp12f_PrintStatus();
    }
    else
    {
        return 0U;
    }
    return 1U;
}
