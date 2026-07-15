#include "debug_cmd_param.h"

#include "adc_monitor.h"
#include "chassis_maintenance_service.h"
#include "debug_console_parser.h"
#include "debug_console_writer.h"
#include "flash_param.h"
#include "imu_bmi270.h"
#include "param_service.h"

#include <stdio.h>
#include <string.h>

#define DEBUG_CMD_PARAM_TX_SIZE 256U

#define PARAM_LOG(level, fmt, ...)                                                                                     \
    do                                                                                                                 \
    {                                                                                                                  \
        char tx[DEBUG_CMD_PARAM_TX_SIZE];                                                                              \
        (void)snprintf(tx, sizeof(tx), "[" level "] " fmt "\r\n", ##__VA_ARGS__);                                      \
        DebugConsoleWriter_Write(tx);                                                                                  \
    } while (0)

static uint8_t DebugCmdParam_SetDirection(const char *line, uint8_t encoder)
{
    char          motor_name[4];
    int           direction;
    motor_id_t    motor;
    param_model_t params;

    if (sscanf(line, encoder ? "set encoder_dir %3s %d" : "set motor_dir %3s %d", motor_name, &direction) != 2
        || DebugConsoleParser_ParseMotor(motor_name, &motor) == 0U || (direction != -1 && direction != 1))
    {
        return 0U;
    }
    if (ChassisMaintenanceService_Begin() != CHASSIS_MAINTENANCE_SERVICE_OK)
    {
        PARAM_LOG("WARN", "direction change rejected: chassis not stationary");
        return 1U;
    }
    ParamService_Get(&params);
    if (encoder != 0U)
    {
        params.encoder_dir[motor] = (int8_t)direction;
    }
    else
    {
        params.motor_dir[motor] = (int8_t)direction;
    }
    if (ParamService_Set(&params) != 0U)
    {
        PARAM_LOG("INFO", "%s_dir %s=%d applied in RAM", encoder ? "encoder" : "motor", motor_name, direction);
    }
    ChassisMaintenanceService_End();
    return 1U;
}

static uint8_t DebugCmdParam_Get(const char *line)
{
    char          param_name[32];
    float         float_value;
    int32_t       int_value;
    param_model_t params;

    if (sscanf(line, "get %31s", param_name) != 1)
    {
        return 0U;
    }
    ParamService_Get(&params);
    if (ParamService_GetFloat(&params, param_name, &float_value) != 0U)
    {
        PARAM_LOG("INFO", "param %s=%.6f", param_name, float_value);
    }
    else if (ParamService_GetInt(&params, param_name, &int_value) != 0U)
    {
        PARAM_LOG("INFO", "param %s=%ld", param_name, (long)int_value);
    }
    else
    {
        PARAM_LOG("ERR", "unknown param");
    }
    return 1U;
}

static uint8_t DebugCmdParam_Save(const char *line)
{
    flash_param_bundle_t bundle;
    param_model_t        params;
    adc_monitor_state_t  adc_state;
    flash_param_status_t status;

    if (strcmp(line, "set save") != 0)
    {
        return 0U;
    }
    if (ChassisMaintenanceService_Begin() != CHASSIS_MAINTENANCE_SERVICE_OK)
    {
        PARAM_LOG("WARN", "param save rejected: chassis not stationary");
        return 1U;
    }
    ParamService_Get(&params);
    AdcMonitor_GetState(&adc_state);
    if (adc_state.current_zero_valid != 0U)
    {
        for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
        {
            params.current_zero_raw[index] = adc_state.current_zero_raw[index];
        }
        params.current_zero_valid = 1U;
    }
    (void)ParamService_Set(&params);
    bundle.params = params;
    ImuBmi270_GetCalibration(&bundle.imu_calibration);
    status = FlashParam_SaveBundle(&bundle);
    if (status == FLASH_PARAM_STATUS_OK)
    {
        PARAM_LOG("INFO", "param saved to flash");
    }
    else
    {
        PARAM_LOG("ERR", "param save failed: %s", FlashParam_StatusString(status));
    }
    ChassisMaintenanceService_End();
    return 1U;
}

static uint8_t DebugCmdParam_Reset(const char *line)
{
    flash_param_bundle_t bundle;
    flash_param_status_t status;

    if (strcmp(line, "set reset") != 0)
    {
        return 0U;
    }
    if (ChassisMaintenanceService_Begin() != CHASSIS_MAINTENANCE_SERVICE_OK)
    {
        PARAM_LOG("WARN", "param reset rejected: chassis not stationary");
        return 1U;
    }
    ParamService_Defaults(&bundle.params);
    ImuBmi270Calibration_Default(&bundle.imu_calibration);
    status = FlashParam_SaveBundle(&bundle);
    if (status == FLASH_PARAM_STATUS_OK)
    {
        ParamService_SetDefaults();
        (void)ImuBmi270_ApplyCalibration(&bundle.imu_calibration);
        ImuBmi270_ClearCalibration();
        AdcMonitor_RequestCurrentZeroCalibration();
        PARAM_LOG("INFO", "param reset defaults saved safely");
    }
    else
    {
        PARAM_LOG("ERR", "param reset failed: %s", FlashParam_StatusString(status));
    }
    ChassisMaintenanceService_End();
    return 1U;
}

static uint8_t DebugCmdParam_Set(const char *line)
{
    char          param_name[32];
    float         param_value;
    param_model_t params;

    if (sscanf(line, "set %31s %f", param_name, &param_value) != 2)
    {
        return 0U;
    }
    if (ChassisMaintenanceService_Begin() != CHASSIS_MAINTENANCE_SERVICE_OK)
    {
        PARAM_LOG("WARN", "param set rejected: chassis not stationary");
        return 1U;
    }
    ParamService_Get(&params);
    if (ParamService_SetFloat(&params, param_name, param_value) != 0U && ParamService_Set(&params) != 0U)
    {
        PARAM_LOG("INFO", "param %s=%.6f", param_name, param_value);
    }
    else if (ParamService_SetInt(&params, param_name, (int32_t)param_value) != 0U && ParamService_Set(&params) != 0U)
    {
        PARAM_LOG("INFO", "param %s=%ld", param_name, (long)(int32_t)param_value);
    }
    else
    {
        PARAM_LOG("ERR", "param set rejected");
    }
    ChassisMaintenanceService_End();
    return 1U;
}

uint8_t DebugCmdParam_TryHandle(char *line)
{
    if (line == 0)
    {
        return 0U;
    }
    if (DebugCmdParam_Get(line) != 0U || DebugCmdParam_Save(line) != 0U || DebugCmdParam_Reset(line) != 0U)
    {
        return 1U;
    }
    if (strncmp(line, "set motor_dir ", 14U) == 0)
    {
        if (DebugCmdParam_SetDirection(line, 0U) == 0U)
        {
            PARAM_LOG("ERR", "usage: set motor_dir m1|m2|m3|m4 -1|1");
        }
        return 1U;
    }
    if (strncmp(line, "set encoder_dir ", 16U) == 0)
    {
        if (DebugCmdParam_SetDirection(line, 1U) == 0U)
        {
            PARAM_LOG("ERR", "usage: set encoder_dir m1|m2|m3|m4 -1|1");
        }
        return 1U;
    }
    return DebugCmdParam_Set(line);
}
