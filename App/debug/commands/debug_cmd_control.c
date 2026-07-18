#include "debug_cmd_control.h"

#include "command_management_service.h"
#include "debug_console_writer.h"
#include "platform_time.h"
#include "motor_config.h"
#include "motion_control_service.h"
#include "motion_control_maintenance.h"
#include "safety_management_service.h"

#include <stdio.h>
#include <string.h>

#define DEBUG_CMD_CONTROL_TX_SIZE 192U

#define CONTROL_LOG(level, fmt, ...)                                                                                   \
    do                                                                                                                 \
    {                                                                                                                  \
        char tx[DEBUG_CMD_CONTROL_TX_SIZE];                                                                            \
        (void)snprintf(tx, sizeof(tx), "[" level "] " fmt "\r\n", ##__VA_ARGS__);                                      \
        DebugConsoleWriter_Write(tx);                                                                                  \
    } while (0)

static int16_t DebugCmdControl_ClampPermille(int32_t value)
{
    if (value > CHASSIS_PWM_MAX_PERMILLE)
    {
        return CHASSIS_PWM_MAX_PERMILLE;
    }
    if (value < -CHASSIS_PWM_MAX_PERMILLE)
    {
        return -CHASSIS_PWM_MAX_PERMILLE;
    }
    return (int16_t)value;
}

static uint8_t DebugCmdControl_ParseMotor(const char *line, uint8_t *motor)
{
    if (line[0] == 'm' && line[1] >= '1' && line[1] <= '4' && line[2] == ' ')
    {
        *motor = (uint8_t)(line[1] - '1');
        return 1U;
    }
    return 0U;
}

static uint8_t DebugCmdControl_TestAllowed(const debug_cmd_control_context_t *context, const char *message)
{
    if (context->motor_test_allowed == 0 || context->motor_test_allowed() == 0U)
    {
        CONTROL_LOG("WARN", "%s", message);
        return 0U;
    }
    return 1U;
}

uint8_t DebugCmdControl_TryHandle(const char *line, const debug_cmd_control_context_t *context)
{
    int     left;
    int     right;
    int     value;
    int     lf;
    int     lr;
    int     rf;
    int     rr;
    int     linear_mm_s;
    int     angular_mrad_s = 0;
    uint8_t motor;

    if (line == 0 || context == 0 || context->velocity_enabled == 0 || context->velocity_generation == 0
        || context->velocity_command == 0)
    {
        return 0U;
    }
    if (sscanf(line, "motor %d %d", &left, &right) == 2)
    {
        if (DebugCmdControl_TestAllowed(context, "motor test rejected: estop/fault active") != 0U)
        {
            *context->velocity_enabled = 0U;
            CommandManagement_ClearAll();
            MotionControl_OpenLoopTest(DebugCmdControl_ClampPermille(left), DebugCmdControl_ClampPermille(right));
            CONTROL_LOG("INFO", "side motor test updated");
        }
    }
    else if (sscanf(line, "left %d", &value) == 1 || sscanf(line, "right %d", &value) == 1)
    {
        uint8_t left_side = (strncmp(line, "left ", 5U) == 0) ? 1U : 0U;
        if (DebugCmdControl_TestAllowed(context,
                                        left_side != 0U ? "left test rejected: estop/fault active"
                                                        : "right test rejected: estop/fault active")
            != 0U)
        {
            *context->velocity_enabled = 0U;
            CommandManagement_ClearAll();
            MotionControl_OpenLoopTest(left_side != 0U ? DebugCmdControl_ClampPermille(value) : 0,
                                       left_side != 0U ? 0 : DebugCmdControl_ClampPermille(value));
            CONTROL_LOG("INFO", "%s side test updated", left_side != 0U ? "left" : "right");
        }
    }
    else if (DebugCmdControl_ParseMotor(line, &motor) != 0U && sscanf(&line[3], "%d %d", &lf, &lr) == 2)
    {
        if (DebugCmdControl_TestAllowed(context, "raw motor test rejected: estop/fault active") != 0U)
        {
            *context->velocity_enabled = 0U;
            CommandManagement_ClearAll();
            MotionControl_RawMotorInputTest(motor,
                                            DebugCmdControl_ClampPermille(lf),
                                            DebugCmdControl_ClampPermille(lr));
            CONTROL_LOG("INFO", "single motor raw test updated");
        }
    }
    else if (sscanf(line, "raw %d %d %d %d", &lf, &lr, &rf, &rr) == 4)
    {
        if (DebugCmdControl_TestAllowed(context, "raw test rejected: estop/fault active") != 0U)
        {
            *context->velocity_enabled = 0U;
            CommandManagement_ClearAll();
            MotionControl_RawInputTest(DebugCmdControl_ClampPermille(lf),
                                       DebugCmdControl_ClampPermille(lr),
                                       DebugCmdControl_ClampPermille(rf),
                                       DebugCmdControl_ClampPermille(rr));
            CONTROL_LOG("INFO", "side raw test updated");
        }
    }
    else if (sscanf(line, "vel %d %d", &linear_mm_s, &angular_mrad_s) == 2 || sscanf(line, "vel %d", &linear_mm_s) == 1)
    {
        uint32_t generation_before = CommandManagement_GetMotionRevokeGeneration();
        *context->velocity_command = (command_velocity_t){
            .linear_x     = (float)linear_mm_s / 1000.0f,
            .angular_z    = (float)angular_mrad_s / 1000.0f,
            .enable       = 1U,
            .source       = COMMAND_SOURCE_DEBUG,
            .timestamp_ms = PlatformTime_TaskNowMs(),
        };
        MotionControl_OpenLoopTest(0, 0);
        if (CommandManagement_SetForGeneration(context->velocity_command, generation_before) == COMMAND_RESULT_ACCEPTED)
        {
            uint32_t generation_after = CommandManagement_GetMotionRevokeGeneration();
            if (generation_before == generation_after && SafetyManagement_IsEmergencyStop() == 0U
                && SafetyManagement_IsFaultStop() == 0U && SafetyManagement_IsMaintenanceLocked() == 0U)
            {
                *context->velocity_generation = generation_after;
                *context->velocity_enabled    = 1U;
                CONTROL_LOG("INFO", "velocity command accepted");
            }
            else
            {
                *context->velocity_enabled = 0U;
                CommandManagement_ClearSource(COMMAND_SOURCE_DEBUG);
                CONTROL_LOG("WARN", "velocity command crossed a safety transition");
            }
        }
        else
        {
            CONTROL_LOG("WARN", "velocity command rejected");
        }
    }
    else if (strcmp(line, "stop") == 0)
    {
        *context->velocity_enabled = 0U;
        MotionControl_OpenLoopTest(0, 0);
        MotionControl_RawInputTest(0, 0, 0, 0);
        CommandManagement_ClearAll();
        CONTROL_LOG("INFO", "chassis stopped");
    }
    else if (sscanf(line, "estop %d", &value) == 1)
    {
        if (value != 0 && context->revoke_maintenance != 0)
        {
            context->revoke_maintenance();
        }
        SafetyManagement_SetEmergencyStop((value != 0) ? 1U : 0U);
        CONTROL_LOG("INFO", "estop %s", (value != 0) ? "set" : "cleared");
    }
    else if (strcmp(line, "clearfault") == 0)
    {
        SafetyManagement_ClearLatchedFaults(0xFFFFFFFFUL);
        CONTROL_LOG("INFO", "fault clear requested");
    }
    else
    {
        return 0U;
    }
    return 1U;
}
