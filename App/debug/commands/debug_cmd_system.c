#include "debug_cmd_system.h"

#include "build_identity.h"
#include "communication_protocol_types.h"
#include "debug_console_parser.h"
#include "debug_console_registry.h"
#include "debug_console_writer.h"
#include "debug_rtos_report.h"
#include "debug_system_status.h"
#include "debug_telemetry.h"
#include "i2c_bus_diagnostic.h"
#include "parameter_management_service.h"

#include <stdio.h>
#include <string.h>

#define DEBUG_CMD_SYSTEM_RX_LINE_SIZE 96U
#define DEBUG_CMD_SYSTEM_TX_SIZE      1536U

static void DebugConsole_PrintHelp(void);
static void DebugConsole_PrintVersion(void);
static void DebugConsole_PrintConfigExport(void);

static debug_cmd_result_t
DebugConsole_CommandHelp(const debug_cmd_context_t *context, int argc, const char *const argv[])
{
    (void)context;
    (void)argv;
    if (argc != 1)
    {
        return DEBUG_CMD_NOT_SUPPORTED;
    }
    DebugConsole_PrintHelp();
    return DEBUG_CMD_OK;
}

static debug_cmd_result_t
DebugConsole_CommandStatus(const debug_cmd_context_t *context, int argc, const char *const argv[])
{
    (void)context;
    (void)argv;
    if (argc != 1)
    {
        return DEBUG_CMD_NOT_SUPPORTED;
    }
    DebugSystemStatus_Print();
    return DEBUG_CMD_OK;
}

static debug_cmd_result_t
DebugConsole_CommandVersion(const debug_cmd_context_t *context, int argc, const char *const argv[])
{
    (void)context;
    (void)argv;
    if (argc != 1)
    {
        return DEBUG_CMD_NOT_SUPPORTED;
    }
    DebugConsole_PrintVersion();
    return DEBUG_CMD_OK;
}

static debug_cmd_result_t
DebugConsole_CommandConfig(const debug_cmd_context_t *context, int argc, const char *const argv[])
{
    (void)context;
    if (argc != 2 || strcmp(argv[1], "export") != 0)
    {
        return DEBUG_CMD_NOT_SUPPORTED;
    }
    DebugConsole_PrintConfigExport();
    return DEBUG_CMD_OK;
}

static debug_cmd_result_t
DebugConsole_CommandRtos(const debug_cmd_context_t *context, int argc, const char *const argv[])
{
    (void)context;
    (void)argv;
    if (argc != 1)
    {
        return DEBUG_CMD_NOT_SUPPORTED;
    }
    DebugRtosReport_Print();
    return DEBUG_CMD_OK;
}

static debug_cmd_result_t
DebugConsole_CommandHeader(const debug_cmd_context_t *context, int argc, const char *const argv[])
{
    (void)context;
    (void)argv;
    if (argc != 1)
    {
        return DEBUG_CMD_NOT_SUPPORTED;
    }
    DebugTelemetry_PrintHeader();
    return DEBUG_CMD_OK;
}

static void DebugCmdSystem_PrintI2cScan(void)
{
    char    tx[DEBUG_CMD_SYSTEM_TX_SIZE];
    int     pos   = 0;
    uint8_t found = 0U;

    pos += snprintf(tx + pos, sizeof(tx) - (size_t)pos, "I2C1 scan:\r\n");
    for (uint8_t address = 1U; address < 128U; ++address)
    {
        if (I2cBusDiagnostic_Probe(address) != 0U)
        {
            found = 1U;
            pos += snprintf(tx + pos, sizeof(tx) - (size_t)pos, "  0x%02X (7-bit)  ACK\r\n", address);
        }
    }
    if (found == 0U)
    {
        (void)snprintf(tx + pos, sizeof(tx) - (size_t)pos, "  no device found\r\n");
    }
    DebugConsoleWriter_Write(tx);
}

static debug_cmd_result_t
DebugConsole_CommandI2cScan(const debug_cmd_context_t *context, int argc, const char *const argv[])
{
    (void)context;
    (void)argv;
    if (argc != 1)
    {
        return DEBUG_CMD_NOT_SUPPORTED;
    }
    DebugCmdSystem_PrintI2cScan();
    return DEBUG_CMD_OK;
}

static const debug_command_t debug_system_commands[] = {
    {"help", "help", "show command help", 0U, DebugConsole_CommandHelp},
    {"h", "h", "show command help", 0U, DebugConsole_CommandHelp},
    {"status", "status", "show system status", 0U, DebugConsole_CommandStatus},
    {"s", "s", "show system status", 0U, DebugConsole_CommandStatus},
    {"version", "version", "show firmware identity", 0U, DebugConsole_CommandVersion},
    {"config", "config export", "export runtime configuration", 0U, DebugConsole_CommandConfig},
    {"rtos", "rtos", "show RTOS task state", 0U, DebugConsole_CommandRtos},
    {"header", "header", "show telemetry CSV header", 0U, DebugConsole_CommandHeader},
    {"i2cscan", "i2cscan", "scan I2C1 addresses", 0U, DebugConsole_CommandI2cScan},
};

uint8_t DebugCmdSystem_TryHandle(const char *line)
{
    char                   storage[DEBUG_CMD_SYSTEM_RX_LINE_SIZE];
    debug_console_args_t   args;
    debug_cmd_context_t    context = {0};
    const debug_command_t *command;

    if (DebugConsoleParser_Parse(line, storage, sizeof(storage), &args) == 0U)
    {
        return 0U;
    }
    command = DebugConsoleRegistry_Find(debug_system_commands,
                                        sizeof(debug_system_commands) / sizeof(debug_system_commands[0]),
                                        args.argv[0]);
    if (command == 0)
    {
        return 0U;
    }
    return (command->handler(&context, args.argc, args.argv) == DEBUG_CMD_OK) ? 1U : 0U;
}

static void DebugConsole_PrintHelp(void)
{
    DebugConsoleWriter_Write("\r\nF407 V2 debug console\r\n"
                             "help/status/header\r\n"
                             "version | config export\r\n"
                             "get <param> | set <param> <value> | set save | set reset\r\n"
                             "set motor_dir m1|m2|m3|m4 -1|1\r\n"
                             "set encoder_dir m1|m2|m3|m4 -1|1\r\n"
                             "maint arm|off          Release raw/open-loop authorization (60s)\r\n"
                             "log 0                  stop streaming\r\n"
                             "log 1 [fld...]         start CSV stream, optional field filter\r\n"
                             "log rate <50..5000> | log csv|json\r\n"
                             "                       fields: motor adc imu errors source ps2 line esp\r\n"
                             "rtos                   heap and task stack status\r\n"
                             "motor L R              side open-loop permille\r\n"
                             "left P/right P         side open-loop shortcut\r\n"
                             "m1 F R ... m4 F R      raw EN/PH, signed PWM = F-R\r\n"
                             "raw LF LR RF RR         left/right EN/PH raw inputs\r\n"
                             "vel V [W]              closed-loop mm/s, optional mrad/s\r\n"
                             "adccal show|zero|plan mN known_mA\r\n"
                             "stop                   clear tests and commands\r\n"
                             "estop 0|1              clear/set emergency stop\r\n"
                             "clearfault             clear latched overcurrent/driver faults\r\n"
                             "line/line on/off       line sensor raw data / toggle control\r\n"
                             "linecal floor|line N | show|apply|cancel\r\n"
                             "imutest/imudiag/imuinit/imucal [n]/imucalclear/imu 0|1\r\n"
                             "espreset/espboot 0|1/espisolate\r\n"
                             "espflash on|off|status bridge USART1 to ESP12F (download mode)\r\n"
                             "espat on|off          bridge USART1 to ESP12F (normal/AT mode)\r\n"
                             "i2cscan               scan I2C1 bus for devices\r\n"
                             "\r\n");
}

static void DebugConsole_PrintVersion(void)
{
    char tx[DEBUG_CMD_SYSTEM_TX_SIZE];

    (void)snprintf(tx,
                   sizeof(tx),
                   "[INFO] version fw=%s sha=%s%s build=%s protocol=%u param=%lu diagnostic=%u\r\n",
                   F407_FIRMWARE_VERSION,
                   F407_GIT_SHA,
                   F407_BUILD_DIRTY ? "-dirty" : "",
                   F407_BUILD_TYPE,
                   COMMUNICATION_PROTOCOL_VERSION,
                   (unsigned long)PARAMETER_MANAGEMENT_VERSION,
                   COMMUNICATION_DIAGNOSTIC_SCHEMA_VERSION);
    DebugConsoleWriter_Write(tx);
}

static void DebugConsole_PrintConfigExport(void)
{
    char          tx[DEBUG_CMD_SYSTEM_TX_SIZE];
    param_model_t params;

    (void)ParameterManagement_GetSnapshot(&params);
    (void)snprintf(tx,
                   sizeof(tx),
                   "{\"param_version\":%lu,\"wheel_radius_m\":%.6f,\"track_width_m\":%.6f,"
                   "\"motor_dir\":[%d,%d,%d,%d],\"encoder_dir\":[%d,%d,%d,%d],"
                   "\"line_threshold\":[%u,%u,%u,%u,%u,%u,%u,%u],\"line_active_low\":%u,"
                   "\"line_kp\":%.6f,\"line_kd\":%.6f,\"line_speed_mps\":%.6f,"
                   "\"current_observe_a\":[%.3f,%.3f,%.3f,%.3f],"
                   "\"current_soft_limit_a\":[%.3f,%.3f,%.3f,%.3f],"
                   "\"current_fault_a\":[%.3f,%.3f,%.3f,%.3f],\"current_debounce_ms\":%u,"
                   "\"straight_wheel_coupling_gain\":%.6f,\"straight_heading_kp\":%.6f,"
                   "\"straight_trim_forward_015_mps\":%.6f,\"straight_trim_forward_030_mps\":%.6f,"
                   "\"straight_trim_reverse_015_mps\":%.6f,\"straight_trim_reverse_030_mps\":%.6f,"
                   "\"straight_heading_ki\":%.6f,\"straight_heading_integral_limit_deg_s\":%.6f,"
                   "\"straight_max_speed_mps\":%.6f,"
                   "\"straight_heading_hold_enabled\":%u}\r\n",
                   (unsigned long)params.version,
                   params.wheel_radius_m,
                   params.track_width_m,
                   params.motor_dir[0],
                   params.motor_dir[1],
                   params.motor_dir[2],
                   params.motor_dir[3],
                   params.encoder_dir[0],
                   params.encoder_dir[1],
                   params.encoder_dir[2],
                   params.encoder_dir[3],
                   params.line_threshold_raw[0],
                   params.line_threshold_raw[1],
                   params.line_threshold_raw[2],
                   params.line_threshold_raw[3],
                   params.line_threshold_raw[4],
                   params.line_threshold_raw[5],
                   params.line_threshold_raw[6],
                   params.line_threshold_raw[7],
                   params.line_active_low,
                   params.line_kp,
                   params.line_kd,
                   params.line_speed_mps,
                   params.current_observe_a[0],
                   params.current_observe_a[1],
                   params.current_observe_a[2],
                   params.current_observe_a[3],
                   params.current_soft_limit_a[0],
                   params.current_soft_limit_a[1],
                   params.current_soft_limit_a[2],
                   params.current_soft_limit_a[3],
                   params.current_fault_a[0],
                   params.current_fault_a[1],
                   params.current_fault_a[2],
                   params.current_fault_a[3],
                   params.current_fault_debounce_ms,
                   params.straight_wheel_coupling_gain,
                   params.straight_heading_kp,
                   params.straight_trim_forward_015_mps,
                   params.straight_trim_forward_030_mps,
                   params.straight_trim_reverse_015_mps,
                   params.straight_trim_reverse_030_mps,
                   params.straight_heading_ki,
                   params.straight_heading_integral_limit_deg_s,
                   params.straight_max_speed_mps,
                   params.straight_heading_hold_enabled);
    DebugConsoleWriter_Write(tx);
}
