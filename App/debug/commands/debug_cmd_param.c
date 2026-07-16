#include "debug_cmd_param.h"

#include "motion_control_service.h"
#include "debug_console_parser.h"
#include "debug_console_writer.h"
#include "parameter_management_service.h"
#include "power_management_service.h"
#include "state_estimation_service.h"

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
    if (MotionControl_BeginMaintenance() != MOTION_CONTROL_MAINTENANCE_OK)
    {
        PARAM_LOG("WARN", "direction change rejected: chassis not stationary");
        return 1U;
    }
    (void)ParameterManagement_GetSnapshot(&params);
    if (encoder != 0U)
    {
        params.encoder_dir[motor] = (int8_t)direction;
    }
    else
    {
        params.motor_dir[motor] = (int8_t)direction;
    }
    if (ParameterManagement_Set(&params) != 0U)
    {
        PARAM_LOG("INFO", "%s_dir %s=%d applied in RAM", encoder ? "encoder" : "motor", motor_name, direction);
    }
    MotionControl_EndMaintenance();
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
    (void)ParameterManagement_GetSnapshot(&params);
    if (ParameterManagement_GetFloat(&params, param_name, &float_value) != 0U)
    {
        PARAM_LOG("INFO", "param %s=%.6f", param_name, float_value);
    }
    else if (ParameterManagement_GetInt(&params, param_name, &int_value) != 0U)
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
    param_model_t             params;
    power_management_status_t power;
    imu_calibration_t         calibration;

    if (strcmp(line, "set save") != 0)
    {
        return 0U;
    }
    if (MotionControl_BeginMaintenance() != MOTION_CONTROL_MAINTENANCE_OK)
    {
        PARAM_LOG("WARN", "param save rejected: chassis not stationary");
        return 1U;
    }
    (void)ParameterManagement_GetSnapshot(&params);
    (void)PowerManagement_GetStatus(&power);
    if (power.current_zero_valid != 0U)
    {
        for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
        {
            params.current_zero_raw[index] = power.current_zero_raw[index];
        }
        params.current_zero_valid = 1U;
    }
    (void)ParameterManagement_Set(&params);
    StateEstimation_GetImuCalibration(&calibration);
    ParameterManagement_SetImuCalibration(&calibration);
    if (ParameterManagement_Save() != 0U)
    {
        PARAM_LOG("INFO", "param saved to flash");
    }
    else
    {
        PARAM_LOG("ERR", "param save failed");
    }
    MotionControl_EndMaintenance();
    return 1U;
}

static uint8_t DebugCmdParam_Reset(const char *line)
{
    imu_calibration_t calibration;

    if (strcmp(line, "set reset") != 0)
    {
        return 0U;
    }
    if (MotionControl_BeginMaintenance() != MOTION_CONTROL_MAINTENANCE_OK)
    {
        PARAM_LOG("WARN", "param reset rejected: chassis not stationary");
        return 1U;
    }
    if (ParameterManagement_ResetAndSaveDefaults() != 0U)
    {
        ParameterImuCalibration_Default(&calibration);
        (void)StateEstimation_ApplyImuCalibration(&calibration);
        StateEstimation_ClearImuCalibration();
        PowerManagement_RequestCurrentZeroCalibration();
        PARAM_LOG("INFO", "param reset defaults saved safely");
    }
    else
    {
        PARAM_LOG("ERR", "param reset failed");
    }
    MotionControl_EndMaintenance();
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
    if (MotionControl_BeginMaintenance() != MOTION_CONTROL_MAINTENANCE_OK)
    {
        PARAM_LOG("WARN", "param set rejected: chassis not stationary");
        return 1U;
    }
    (void)ParameterManagement_GetSnapshot(&params);
    if (ParameterManagement_SetFloat(&params, param_name, param_value) != 0U && ParameterManagement_Set(&params) != 0U)
    {
        PARAM_LOG("INFO", "param %s=%.6f", param_name, param_value);
    }
    else if (ParameterManagement_SetInt(&params, param_name, (int32_t)param_value) != 0U
             && ParameterManagement_Set(&params) != 0U)
    {
        PARAM_LOG("INFO", "param %s=%ld", param_name, (long)(int32_t)param_value);
    }
    else
    {
        PARAM_LOG("ERR", "param set rejected");
    }
    MotionControl_EndMaintenance();
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
