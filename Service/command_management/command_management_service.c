#include "command_management_service.h"

#include "parameter_management_service.h"
#include "platform_critical.h"

static command_management_config_t command_config;
static command_velocity_t          source_commands[COMMAND_SOURCE_COUNT];
static uint8_t                     motion_allowed;
static uint8_t                     command_initialized;
static uint32_t                    motion_revoke_generation;
static uint32_t                    motion_gate_generation;
static uint32_t                    command_generation;
static uint8_t                     rearm_required_mask;
static uint32_t                    slot_generation[COMMAND_SOURCE_COUNT];

#define COMMAND_SOURCE_MASK(source) ((uint8_t)(1U << (source)))
#define COMMAND_ALL_SOURCE_MASK                                                                                        \
    ((uint8_t)(COMMAND_SOURCE_MASK(COMMAND_SOURCE_HOST) | COMMAND_SOURCE_MASK(COMMAND_SOURCE_PS2)                      \
               | COMMAND_SOURCE_MASK(COMMAND_SOURCE_ESP12F) | COMMAND_SOURCE_MASK(COMMAND_SOURCE_DEBUG)                \
               | COMMAND_SOURCE_MASK(COMMAND_SOURCE_LINE)))

static const command_source_t source_priority[] = {
    COMMAND_SOURCE_HOST,
    COMMAND_SOURCE_PS2,
    COMMAND_SOURCE_ESP12F,
    COMMAND_SOURCE_LINE,
    COMMAND_SOURCE_DEBUG,
};

static uint8_t CommandManagement_SourceValid(command_source_t source)
{
    return (source > COMMAND_SOURCE_NONE && source < COMMAND_SOURCE_COUNT) ? 1U : 0U;
}

static uint8_t CommandManagement_IsRemoteSource(command_source_t source)
{
    return (source == COMMAND_SOURCE_HOST || source == COMMAND_SOURCE_ESP12F) ? 1U : 0U;
}

static uint8_t CommandManagement_Finite(float value)
{
    const float max_float = 3.402823466e+38f;

    return (value == value && value <= max_float && value >= -max_float) ? 1U : 0U;
}

static float CommandManagement_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float CommandManagement_Clamp(float value, float limit)
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

static uint8_t CommandManagement_ConfigValid(const command_management_config_t *config)
{
    return (config != 0 && config->host_timeout_ms > 0UL && config->ps2_timeout_ms > 0UL
            && config->esp12f_timeout_ms > 0UL && config->line_timeout_ms > 0UL && config->debug_timeout_ms > 0UL)
               ? 1U
               : 0U;
}

static uint32_t CommandManagement_Timeout(command_source_t source)
{
    switch (source)
    {
        case COMMAND_SOURCE_HOST:
            return command_config.host_timeout_ms;
        case COMMAND_SOURCE_PS2:
            return command_config.ps2_timeout_ms;
        case COMMAND_SOURCE_ESP12F:
            return command_config.esp12f_timeout_ms;
        case COMMAND_SOURCE_LINE:
            return command_config.line_timeout_ms;
        case COMMAND_SOURCE_DEBUG:
            return command_config.debug_timeout_ms;
        default:
            return 0UL;
    }
}

uint8_t CommandManagement_Init(const command_management_config_t *config)
{
    platform_critical_state_t critical;

    if (CommandManagement_ConfigValid(config) == 0U)
    {
        return 0U;
    }
    critical       = PlatformCritical_Enter();
    command_config = *config;
    for (uint8_t index = 0U; index < COMMAND_SOURCE_COUNT; ++index)
    {
        source_commands[index] = (command_velocity_t){0};
    }
    motion_allowed           = 0U;
    command_initialized      = 1U;
    motion_revoke_generation = 0UL;
    motion_gate_generation   = 0UL;
    command_generation       = 1UL;
    rearm_required_mask      = COMMAND_ALL_SOURCE_MASK;
    for (uint8_t index = 0U; index < COMMAND_SOURCE_COUNT; ++index)
    {
        slot_generation[index] = 1UL;
    }
    PlatformCritical_Exit(critical);
    return 1U;
}

uint8_t CommandManagement_IsInitialized(void)
{
    return command_initialized;
}

void CommandManagement_ClearAll(void)
{
    platform_critical_state_t critical = PlatformCritical_Enter();

    for (uint8_t index = 0U; index < COMMAND_SOURCE_COUNT; ++index)
    {
        source_commands[index] = (command_velocity_t){0};
    }
    command_generation++;
    PlatformCritical_Exit(critical);
}

void CommandManagement_ClearSource(command_source_t source)
{
    platform_critical_state_t critical;

    if (CommandManagement_SourceValid(source) == 0U)
    {
        return;
    }
    critical                = PlatformCritical_Enter();
    source_commands[source] = (command_velocity_t){0};
    slot_generation[source]++;
    command_generation++;
    PlatformCritical_Exit(critical);
}

command_result_t CommandManagement_DisableRemoteSource(command_source_t source)
{
    if (CommandManagement_IsRemoteSource(source) == 0U)
    {
        return COMMAND_RESULT_REJECTED;
    }
    return (CommandManagement_QualifyRearm(source).outcome == COMMAND_OUTCOME_RELEASE_ACCEPTED)
               ? COMMAND_RESULT_ACCEPTED
               : COMMAND_RESULT_REJECTED;
}

command_apply_result_t CommandManagement_QualifyRearm(command_source_t source)
{
    command_apply_result_t    result = {.outcome = COMMAND_OUTCOME_INVALID};
    platform_critical_state_t critical;

    if (CommandManagement_SourceValid(source) == 0U)
    {
        return result;
    }
    critical                = PlatformCritical_Enter();
    source_commands[source] = (command_velocity_t){0};
    slot_generation[source]++;
    command_generation++;
    result.source_cleared  = 1U;
    result.slot_generation = slot_generation[source];
    if (motion_allowed == 0U)
    {
        result.outcome = COMMAND_OUTCOME_GATE_CLOSED;
    }
    else
    {
        rearm_required_mask &= (uint8_t)~COMMAND_SOURCE_MASK(source);
        result.outcome = COMMAND_OUTCOME_RELEASE_ACCEPTED;
    }
    PlatformCritical_Exit(critical);
    return result;
}

uint8_t CommandManagement_RefreshSource(command_source_t source, uint32_t now_ms)
{
    platform_critical_state_t critical;
    command_velocity_t       *command;

    if (CommandManagement_SourceValid(source) == 0U)
    {
        return 0U;
    }
    critical = PlatformCritical_Enter();
    command  = &source_commands[source];
    if (motion_allowed == 0U || command->enable == 0U
        || (uint32_t)(now_ms - command->timestamp_ms) > CommandManagement_Timeout(source))
    {
        PlatformCritical_Exit(critical);
        return 0U;
    }
    command->timestamp_ms = now_ms;
    PlatformCritical_Exit(critical);
    return 1U;
}

static command_apply_result_t CommandManagement_ApplyInternal(const command_velocity_t *command,
                                                              uint8_t                   enforce_generation,
                                                              uint32_t                  expected_generation)
{
    command_velocity_t        sanitized;
    param_model_t             params;
    platform_critical_state_t critical;

    if (command == 0 || CommandManagement_SourceValid(command->source) == 0U)
    {
        return (command_apply_result_t){.outcome = COMMAND_OUTCOME_INVALID};
    }
    sanitized = *command;
    critical  = PlatformCritical_Enter();
    if (motion_allowed == 0U)
    {
        PlatformCritical_Exit(critical);
        return (command_apply_result_t){.outcome         = COMMAND_OUTCOME_GATE_CLOSED,
                                        .slot_generation = slot_generation[sanitized.source]};
    }
    if (enforce_generation != 0U && motion_revoke_generation != expected_generation)
    {
        PlatformCritical_Exit(critical);
        return (command_apply_result_t){.outcome         = COMMAND_OUTCOME_GENERATION_CONFLICT,
                                        .slot_generation = slot_generation[sanitized.source]};
    }
    PlatformCritical_Exit(critical);
    if (sanitized.enable == 0U)
    {
        return CommandManagement_QualifyRearm(sanitized.source);
    }
    critical = PlatformCritical_Enter();
    if ((rearm_required_mask & COMMAND_SOURCE_MASK(sanitized.source)) != 0U)
    {
        PlatformCritical_Exit(critical);
        return (command_apply_result_t){.outcome         = COMMAND_OUTCOME_REARM_REQUIRED,
                                        .slot_generation = slot_generation[sanitized.source]};
    }
    PlatformCritical_Exit(critical);
    if (ParameterManagement_GetSnapshot(&params) == 0UL)
    {
        return (command_apply_result_t){.outcome = COMMAND_OUTCOME_INVALID};
    }
    if (CommandManagement_Finite(sanitized.linear_x) == 0U || CommandManagement_Finite(sanitized.angular_z) == 0U)
    {
        CommandManagement_ClearSource(sanitized.source);
        return (command_apply_result_t){.outcome         = COMMAND_OUTCOME_INVALID,
                                        .source_cleared  = 1U,
                                        .slot_generation = slot_generation[sanitized.source]};
    }
    sanitized.linear_x  = CommandManagement_Clamp(sanitized.linear_x, params.max_linear_mps);
    sanitized.angular_z = CommandManagement_Clamp(sanitized.angular_z, params.max_angular_rps);
    if ((params.wheel_radius_m <= 0.0f || params.track_width_m <= 0.0f)
        && CommandManagement_Abs(sanitized.angular_z) > 0.0001f)
    {
        CommandManagement_ClearSource(sanitized.source);
        return (command_apply_result_t){.outcome         = COMMAND_OUTCOME_INVALID,
                                        .source_cleared  = 1U,
                                        .slot_generation = slot_generation[sanitized.source]};
    }

    critical = PlatformCritical_Enter();
    if (motion_allowed == 0U || (rearm_required_mask & COMMAND_SOURCE_MASK(sanitized.source)) != 0U
        || (enforce_generation != 0U && motion_revoke_generation != expected_generation))
    {
        PlatformCritical_Exit(critical);
        return (command_apply_result_t){.outcome         = (motion_allowed == 0U) ? COMMAND_OUTCOME_GATE_CLOSED
                                                                                  : COMMAND_OUTCOME_GENERATION_CONFLICT,
                                        .slot_generation = slot_generation[sanitized.source]};
    }
    source_commands[sanitized.source] = sanitized;
    command_generation++;
    slot_generation[sanitized.source]++;
    command_apply_result_t result = {.outcome         = COMMAND_OUTCOME_ACTIVE_ACCEPTED,
                                     .slot_generation = slot_generation[sanitized.source]};
    PlatformCritical_Exit(critical);
    return result;
}

command_apply_result_t CommandManagement_Apply(const command_velocity_t *command)
{
    return CommandManagement_ApplyInternal(command, 0U, 0UL);
}

command_apply_result_t CommandManagement_ApplyForGeneration(const command_velocity_t *command,
                                                            uint32_t                  expected_generation)
{
    return CommandManagement_ApplyInternal(command, 1U, expected_generation);
}

static command_result_t CommandManagement_LegacyResult(command_apply_result_t result)
{
    if (result.outcome == COMMAND_OUTCOME_ACTIVE_ACCEPTED || result.outcome == COMMAND_OUTCOME_RELEASE_ACCEPTED)
    {
        return COMMAND_RESULT_ACCEPTED;
    }
    return (result.source_cleared != 0U) ? COMMAND_RESULT_REJECTED_AND_STOPPED : COMMAND_RESULT_REJECTED;
}

command_result_t CommandManagement_Set(const command_velocity_t *command)
{
    return CommandManagement_LegacyResult(CommandManagement_Apply(command));
}

command_result_t CommandManagement_SetForGeneration(const command_velocity_t *command, uint32_t expected_generation)
{
    return CommandManagement_LegacyResult(CommandManagement_ApplyForGeneration(command, expected_generation));
}

uint8_t CommandManagement_GetActive(command_velocity_t *command, uint32_t now_ms)
{
    platform_critical_state_t critical;

    if (command != 0)
    {
        *command = (command_velocity_t){0};
    }
    critical = PlatformCritical_Enter();
    if (motion_allowed == 0U)
    {
        PlatformCritical_Exit(critical);
        return 0U;
    }
    for (uint8_t index = 0U; index < (uint8_t)(sizeof(source_priority) / sizeof(source_priority[0])); ++index)
    {
        command_velocity_t snapshot = source_commands[source_priority[index]];

        if (snapshot.enable != 0U && CommandManagement_SourceValid(snapshot.source) != 0U
            && (uint32_t)(now_ms - snapshot.timestamp_ms) <= CommandManagement_Timeout(snapshot.source))
        {
            if (command != 0)
            {
                *command = snapshot;
            }
            PlatformCritical_Exit(critical);
            return 1U;
        }
    }
    PlatformCritical_Exit(critical);
    return 0U;
}

command_source_t CommandManagement_GetActiveSource(uint32_t now_ms)
{
    command_velocity_t command;

    return (CommandManagement_GetActive(&command, now_ms) != 0U) ? command.source : COMMAND_SOURCE_NONE;
}

uint32_t CommandManagement_GetMotionRevokeGeneration(void)
{
    platform_critical_state_t critical;
    uint32_t                  generation;

    critical   = PlatformCritical_Enter();
    generation = motion_revoke_generation;
    PlatformCritical_Exit(critical);
    return generation;
}

void CommandManagement_SetMotionGate(uint8_t allowed, uint32_t decision_generation)
{
    uint8_t                   next_allowed = (allowed != 0U) ? 1U : 0U;
    platform_critical_state_t critical     = PlatformCritical_Enter();

    if ((int32_t)(decision_generation - motion_gate_generation) < 0)
    {
        PlatformCritical_Exit(critical);
        return;
    }
    motion_gate_generation = decision_generation;
    if (next_allowed == 0U)
    {
        if (motion_allowed != 0U)
        {
            motion_revoke_generation++;
        }
        for (uint8_t index = 0U; index < COMMAND_SOURCE_COUNT; ++index)
        {
            source_commands[index] = (command_velocity_t){0};
        }
        rearm_required_mask |= COMMAND_ALL_SOURCE_MASK;
        for (uint8_t index = 0U; index < COMMAND_SOURCE_COUNT; ++index)
        {
            slot_generation[index]++;
        }
    }
    motion_allowed = next_allowed;
    command_generation++;
    PlatformCritical_Exit(critical);
}

uint8_t CommandManagement_IsMotionGateOpen(void)
{
    platform_critical_state_t critical;
    uint8_t                   allowed;

    critical = PlatformCritical_Enter();
    allowed  = motion_allowed;
    PlatformCritical_Exit(critical);
    return allowed;
}

uint32_t CommandManagement_GetStatus(uint32_t now_ms, command_management_status_t *status)
{
    platform_critical_state_t critical;

    if (status == 0)
    {
        return 0UL;
    }
    critical              = PlatformCritical_Enter();
    status->active_source = COMMAND_SOURCE_NONE;
    if (motion_allowed != 0U)
    {
        for (uint8_t index = 0U; index < (uint8_t)(sizeof(source_priority) / sizeof(source_priority[0])); ++index)
        {
            command_velocity_t snapshot = source_commands[source_priority[index]];

            if (snapshot.enable != 0U && CommandManagement_SourceValid(snapshot.source) != 0U
                && (uint32_t)(now_ms - snapshot.timestamp_ms) <= CommandManagement_Timeout(snapshot.source))
            {
                status->active_source = snapshot.source;
                break;
            }
        }
    }
    status->motion_allowed           = motion_allowed;
    status->rearm_required_mask      = rearm_required_mask;
    status->motion_revoke_generation = motion_revoke_generation;
    status->generation               = command_generation;
    PlatformCritical_Exit(critical);
    return status->generation;
}
