#include "control_service.h"
#include "platform_critical.h"
#include "platform_time.h"

#include "control_config.h"

#include "param_service.h"

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

static uint8_t ControlService_IsFiniteFloat(float value)
{
    const float max_float = 3.402823466e+38f;
    return (value == value && value <= max_float && value >= -max_float) ? 1U : 0U;
}

static float ControlService_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float ControlService_ClampFloat(float value, float limit)
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

static uint8_t ControlService_KinematicsValid(const param_model_t *params)
{
    return (params != 0 && params->wheel_radius_m > 0.0f && params->track_width_m > 0.0f) ? 1U : 0U;
}

void ControlService_Init(void)
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

void ControlService_ClearCommand(void)
{
    uint32_t primask = PlatformCritical_Enter();
    for (uint8_t i = 0U; i <= CONTROL_SOURCE_LINE; ++i)
    {
        source_cmds[i] = (chassis_cmd_t){0};
    }
    PlatformCritical_Exit(primask);
}

void ControlService_ClearSource(uint8_t source)
{
    uint32_t primask;

    if (source == CONTROL_SOURCE_NONE || source > CONTROL_SOURCE_LINE)
    {
        return;
    }

    primask             = PlatformCritical_Enter();
    source_cmds[source] = (chassis_cmd_t){0};
    PlatformCritical_Exit(primask);
}

static control_command_result_t
ControlService_SetCommandInternal(const chassis_cmd_t *cmd, uint8_t enforce_generation, uint32_t expected_generation)
{
    if (cmd != 0)
    {
        chassis_cmd_t sanitized = *cmd;
        param_model_t params;
        uint32_t      primask;

        (void)ParamService_GetSnapshot(&params);
        if (ControlService_IsEmergencyStop() != 0U || ControlService_IsFaultStop() != 0U
            || ControlService_IsMaintenanceLocked() != 0U
            || (enforce_generation != 0U && ControlService_GetMotionRevokeGeneration() != expected_generation))
        {
            return CONTROL_COMMAND_REJECTED;
        }
        if (sanitized.source == CONTROL_SOURCE_NONE || sanitized.source > CONTROL_SOURCE_LINE)
        {
            return CONTROL_COMMAND_REJECTED;
        }

        if (ControlService_IsFiniteFloat(sanitized.linear_x) == 0U
            || ControlService_IsFiniteFloat(sanitized.angular_z) == 0U)
        {
            ControlService_ClearSource(sanitized.source);
            return CONTROL_COMMAND_REJECTED_AND_STOPPED;
        }

        if (sanitized.enable == 0U)
        {
            ControlService_ClearSource(sanitized.source);
            return CONTROL_COMMAND_REJECTED_AND_STOPPED;
        }

        sanitized.linear_x  = ControlService_ClampFloat(sanitized.linear_x, params.max_linear_mps);
        sanitized.angular_z = ControlService_ClampFloat(sanitized.angular_z, params.max_angular_rps);
        if (ControlService_KinematicsValid(&params) == 0U
            && ControlService_AbsFloat(sanitized.angular_z) > CHASSIS_ANGULAR_EPSILON_RPS)
        {
            ControlService_ClearSource(sanitized.source);
            return CONTROL_COMMAND_REJECTED_AND_STOPPED;
        }

        primask = PlatformCritical_Enter();
        if (emergency_stop != 0U || fault_stop != 0U || maintenance_lock != 0U
            || (enforce_generation != 0U && motion_revoke_generation != expected_generation))
        {
            PlatformCritical_Exit(primask);
            return CONTROL_COMMAND_REJECTED;
        }
        source_cmds[sanitized.source] = sanitized;
        PlatformCritical_Exit(primask);
        return CONTROL_COMMAND_ACCEPTED;
    }

    return CONTROL_COMMAND_REJECTED;
}

control_command_result_t ControlService_SetCommand(const chassis_cmd_t *cmd)
{
    return ControlService_SetCommandInternal(cmd, 0U, 0UL);
}

control_command_result_t ControlService_SetCommandForGeneration(const chassis_cmd_t *cmd, uint32_t expected_generation)
{
    return ControlService_SetCommandInternal(cmd, 1U, expected_generation);
}

uint8_t ControlService_BeginMaintenance(void)
{
    uint32_t primask = PlatformCritical_Enter();
    if (maintenance_lock != 0U)
    {
        PlatformCritical_Exit(primask);
        return 0U;
    }
    maintenance_lock = 1U;
    motion_revoke_generation++;
    for (uint8_t i = 0U; i <= CONTROL_SOURCE_LINE; ++i)
    {
        source_cmds[i] = (chassis_cmd_t){0};
    }
    PlatformCritical_Exit(primask);
    return 1U;
}

void ControlService_EndMaintenance(void)
{
    uint32_t primask = PlatformCritical_Enter();
    maintenance_lock = 0U;
    PlatformCritical_Exit(primask);
}

uint8_t ControlService_IsMaintenanceLocked(void)
{
    uint8_t  value;
    uint32_t primask = PlatformCritical_Enter();
    value            = maintenance_lock;
    PlatformCritical_Exit(primask);
    return value;
}

uint32_t ControlService_GetMotionRevokeGeneration(void)
{
    uint32_t generation;
    uint32_t primask = PlatformCritical_Enter();
    generation       = motion_revoke_generation;
    PlatformCritical_Exit(primask);
    return generation;
}

void ControlService_SetEmergencyStop(uint8_t enabled)
{
    uint32_t primask = PlatformCritical_Enter();
    if (enabled != 0U && emergency_stop == 0U)
    {
        motion_revoke_generation++;
    }
    emergency_stop = (enabled != 0U) ? 1U : 0U;
    for (uint8_t i = 0U; i <= CONTROL_SOURCE_LINE; ++i)
    {
        source_cmds[i] = (chassis_cmd_t){0};
    }
    PlatformCritical_Exit(primask);
}

void ControlService_SetFaultStop(uint8_t enabled)
{
    uint32_t primask = PlatformCritical_Enter();
    if (enabled != 0U && fault_stop == 0U)
    {
        motion_revoke_generation++;
    }
    fault_stop = (enabled != 0U) ? 1U : 0U;
    for (uint8_t i = 0U; i <= CONTROL_SOURCE_LINE; ++i)
    {
        source_cmds[i] = (chassis_cmd_t){0};
    }
    PlatformCritical_Exit(primask);
}

uint8_t ControlService_GetCommand(chassis_cmd_t *cmd, uint32_t now_ms)
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
    primask = PlatformCritical_Enter();
    if (emergency_stop != 0U || fault_stop != 0U || maintenance_lock != 0U)
    {
        PlatformCritical_Exit(primask);
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
            PlatformCritical_Exit(primask);
            return 1U;
        }
    }
    PlatformCritical_Exit(primask);
    return 0U;
}

uint8_t ControlService_IsEmergencyStop(void)
{
    uint8_t  value;
    uint32_t primask = PlatformCritical_Enter();
    value            = emergency_stop;
    PlatformCritical_Exit(primask);
    return value;
}

uint8_t ControlService_IsFaultStop(void)
{
    uint8_t  value;
    uint32_t primask = PlatformCritical_Enter();
    value            = fault_stop;
    PlatformCritical_Exit(primask);
    return value;
}

uint8_t ControlService_GetActiveSource(void)
{
    chassis_cmd_t snapshot;

    if (ControlService_GetCommand(&snapshot, PlatformTime_TaskNowMs()) == 0U)
    {
        return CONTROL_SOURCE_NONE;
    }

    return snapshot.source;
}
