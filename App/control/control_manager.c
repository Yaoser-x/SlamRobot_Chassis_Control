#include "control_manager.h"

#include "chassis_config.h"
#include "cmsis_os2.h"
#include "main.h"
#include "param_store.h"

static chassis_cmd_t source_cmds[CONTROL_SOURCE_LINE + 1U];
static uint8_t       emergency_stop;
static uint8_t       fault_stop;
static uint8_t       maintenance_lock;
static uint32_t      motion_revoke_generation;

/* 按源独立超时 (ms)：UPPER/PS2/ESP12F/LINE/DEBUG */
static const uint32_t source_timeout_ms[CONTROL_SOURCE_LINE + 1U] = {
    [CONTROL_SOURCE_NONE]   = 0U,
    [CONTROL_SOURCE_UPPER]  = CONTROL_TIMEOUT_UPPER_MS,
    [CONTROL_SOURCE_PS2]    = CONTROL_TIMEOUT_PS2_MS,
    [CONTROL_SOURCE_ESP12F] = CONTROL_TIMEOUT_ESP12F_MS,
    [CONTROL_SOURCE_LINE]   = CONTROL_TIMEOUT_LINE_MS,
    [CONTROL_SOURCE_DEBUG]  = CONTROL_TIMEOUT_DEBUG_MS,
};

static uint8_t ControlManager_IsFiniteFloat(float value)
{
    const float max_float = 3.402823466e+38f;
    return (value == value && value <= max_float && value >= -max_float) ? 1U : 0U;
}

static float ControlManager_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float ControlManager_ClampFloat(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static uint8_t ControlManager_KinematicsValid(const param_store_t *params)
{
    return (params != 0 && params->wheel_radius_m > 0.0f && params->track_width_m > 0.0f) ? 1U : 0U;
}

void ControlManager_Init(void)
{
    for (uint8_t i = 0U; i <= CONTROL_SOURCE_LINE; ++i)
    {
        source_cmds[i] = (chassis_cmd_t){0};
    }
    emergency_stop           = 0U;
    fault_stop               = 0U;
    maintenance_lock         = 0U;
    motion_revoke_generation = 0UL;
}

void ControlManager_ClearCommand(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    for (uint8_t i = 0U; i <= CONTROL_SOURCE_LINE; ++i)
    {
        source_cmds[i] = (chassis_cmd_t){0};
    }
    __set_PRIMASK(primask);
}

void ControlManager_ClearSource(uint8_t source)
{
    uint32_t primask;

    if (source == CONTROL_SOURCE_NONE || source > CONTROL_SOURCE_LINE)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    source_cmds[source] = (chassis_cmd_t){0};
    __set_PRIMASK(primask);
}

static control_command_result_t
ControlManager_SetCommandInternal(const chassis_cmd_t *cmd, uint8_t enforce_generation, uint32_t expected_generation)
{
    if (cmd != 0)
    {
        chassis_cmd_t sanitized = *cmd;
        param_store_t params;
        uint32_t      primask;

        (void)ParamStore_GetSnapshot(&params);
        if (ControlManager_IsEmergencyStop() != 0U || ControlManager_IsFaultStop() != 0U
            || ControlManager_IsMaintenanceLocked() != 0U
            || (enforce_generation != 0U && ControlManager_GetMotionRevokeGeneration() != expected_generation))
        {
            return CONTROL_COMMAND_REJECTED;
        }
        if (sanitized.source == CONTROL_SOURCE_NONE || sanitized.source > CONTROL_SOURCE_LINE)
        {
            return CONTROL_COMMAND_REJECTED;
        }

        if (ControlManager_IsFiniteFloat(sanitized.linear_x) == 0U
            || ControlManager_IsFiniteFloat(sanitized.angular_z) == 0U)
        {
            ControlManager_ClearSource(sanitized.source);
            return CONTROL_COMMAND_REJECTED_AND_STOPPED;
        }

        if (sanitized.enable == 0U)
        {
            ControlManager_ClearSource(sanitized.source);
            return CONTROL_COMMAND_REJECTED_AND_STOPPED;
        }

        sanitized.linear_x  = ControlManager_ClampFloat(sanitized.linear_x, params.max_linear_mps);
        sanitized.angular_z = ControlManager_ClampFloat(sanitized.angular_z, params.max_angular_rps);
        if (ControlManager_KinematicsValid(&params) == 0U
            && ControlManager_AbsFloat(sanitized.angular_z) > CHASSIS_ANGULAR_EPSILON_RPS)
        {
            ControlManager_ClearSource(sanitized.source);
            return CONTROL_COMMAND_REJECTED_AND_STOPPED;
        }

        primask = __get_PRIMASK();
        __disable_irq();
        if (emergency_stop != 0U || fault_stop != 0U || maintenance_lock != 0U
            || (enforce_generation != 0U && motion_revoke_generation != expected_generation))
        {
            __set_PRIMASK(primask);
            return CONTROL_COMMAND_REJECTED;
        }
        source_cmds[sanitized.source] = sanitized;
        __set_PRIMASK(primask);
        return CONTROL_COMMAND_ACCEPTED;
    }

    return CONTROL_COMMAND_REJECTED;
}

control_command_result_t ControlManager_SetCommand(const chassis_cmd_t *cmd)
{
    return ControlManager_SetCommandInternal(cmd, 0U, 0UL);
}

control_command_result_t ControlManager_SetCommandForGeneration(const chassis_cmd_t *cmd, uint32_t expected_generation)
{
    return ControlManager_SetCommandInternal(cmd, 1U, expected_generation);
}

uint8_t ControlManager_BeginMaintenance(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (maintenance_lock != 0U)
    {
        __set_PRIMASK(primask);
        return 0U;
    }
    maintenance_lock = 1U;
    motion_revoke_generation++;
    for (uint8_t i = 0U; i <= CONTROL_SOURCE_LINE; ++i)
    {
        source_cmds[i] = (chassis_cmd_t){0};
    }
    __set_PRIMASK(primask);
    return 1U;
}

void ControlManager_EndMaintenance(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    maintenance_lock = 0U;
    __set_PRIMASK(primask);
}

uint8_t ControlManager_IsMaintenanceLocked(void)
{
    uint8_t  value;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    value = maintenance_lock;
    __set_PRIMASK(primask);
    return value;
}

uint32_t ControlManager_GetMotionRevokeGeneration(void)
{
    uint32_t generation;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    generation = motion_revoke_generation;
    __set_PRIMASK(primask);
    return generation;
}

void ControlManager_SetEmergencyStop(uint8_t enabled)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (enabled != 0U && emergency_stop == 0U)
    {
        motion_revoke_generation++;
    }
    emergency_stop = (enabled != 0U) ? 1U : 0U;
    for (uint8_t i = 0U; i <= CONTROL_SOURCE_LINE; ++i)
    {
        source_cmds[i] = (chassis_cmd_t){0};
    }
    __set_PRIMASK(primask);
}

void ControlManager_SetFaultStop(uint8_t enabled)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (enabled != 0U && fault_stop == 0U)
    {
        motion_revoke_generation++;
    }
    fault_stop = (enabled != 0U) ? 1U : 0U;
    for (uint8_t i = 0U; i <= CONTROL_SOURCE_LINE; ++i)
    {
        source_cmds[i] = (chassis_cmd_t){0};
    }
    __set_PRIMASK(primask);
}

uint8_t ControlManager_GetCommand(chassis_cmd_t *cmd, uint32_t now_ms)
{
    static const uint8_t source_priority[] = {
        CONTROL_SOURCE_UPPER,
        CONTROL_SOURCE_PS2,
        CONTROL_SOURCE_ESP12F,
        CONTROL_SOURCE_LINE,
        CONTROL_SOURCE_DEBUG,
    };
    uint32_t primask;

    if (cmd != 0)
    {
        *cmd = (chassis_cmd_t){0};
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (emergency_stop != 0U || fault_stop != 0U || maintenance_lock != 0U)
    {
        __set_PRIMASK(primask);
        return 0U;
    }
    for (uint8_t i = 0U; i < (uint8_t)(sizeof(source_priority) / sizeof(source_priority[0])); ++i)
    {
        chassis_cmd_t snapshot = source_cmds[source_priority[i]];
        uint32_t      age_ms   = now_ms - snapshot.timestamp_ms;

        if (snapshot.enable != 0U && snapshot.source != CONTROL_SOURCE_NONE
            && age_ms <= source_timeout_ms[snapshot.source])
        {
            if (cmd != 0)
            {
                *cmd = snapshot;
            }
            __set_PRIMASK(primask);
            return 1U;
        }
    }
    __set_PRIMASK(primask);
    return 0U;
}

uint8_t ControlManager_IsEmergencyStop(void)
{
    uint8_t  value;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    value = emergency_stop;
    __set_PRIMASK(primask);
    return value;
}

uint8_t ControlManager_IsFaultStop(void)
{
    uint8_t  value;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    value = fault_stop;
    __set_PRIMASK(primask);
    return value;
}

uint8_t ControlManager_GetActiveSource(void)
{
    chassis_cmd_t snapshot;

    if (ControlManager_GetCommand(&snapshot, osKernelGetTickCount()) == 0U)
    {
        return CONTROL_SOURCE_NONE;
    }

    return snapshot.source;
}
