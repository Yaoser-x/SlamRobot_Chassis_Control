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
static uint32_t                    mode_generation;
static uint8_t                     permitted_source_mask;
static uint8_t                     rearm_required_mask;
static uint32_t                    slot_generation[COMMAND_SOURCE_COUNT];
static uint32_t                    accepted_command_id[COMMAND_SOURCE_COUNT];
static uint32_t                    accepted_at_ms[COMMAND_SOURCE_COUNT];
static uint8_t                     accepted_mode[COMMAND_SOURCE_COUNT];

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
            && config->esp12f_timeout_ms > 0UL && config->line_timeout_ms > 0UL && config->debug_timeout_ms > 0UL
            && config->remote_max_lifetime_ms >= config->host_timeout_ms
            && config->remote_max_lifetime_ms >= config->esp12f_timeout_ms && config->remote_max_lifetime_ms <= 60000UL)
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
    mode_generation          = 0UL;
    permitted_source_mask    = COMMAND_ALL_SOURCE_MASK;
    rearm_required_mask      = COMMAND_ALL_SOURCE_MASK;
    for (uint8_t index = 0U; index < COMMAND_SOURCE_COUNT; ++index)
    {
        slot_generation[index]     = 1UL;
        accepted_command_id[index] = 0UL;
        accepted_at_ms[index]      = 0UL;
        accepted_mode[index]       = 0U;
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
        slot_generation[index]++;
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
    critical = PlatformCritical_Enter();
    if ((permitted_source_mask & COMMAND_SOURCE_BIT(source)) == 0U)
    {
        result.outcome = COMMAND_OUTCOME_SOURCE_NOT_ALLOWED;
        PlatformCritical_Exit(critical);
        return result;
    }
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
        rearm_required_mask &= (uint8_t)~COMMAND_SOURCE_BIT(source);
        result.outcome = COMMAND_OUTCOME_RELEASE_ACCEPTED;
    }
    PlatformCritical_Exit(critical);
    return result;
}

uint8_t CommandManagement_GetRefreshToken(command_source_t         source,
                                          uint32_t                 expected_slot_generation,
                                          command_refresh_token_t *token)
{
    platform_critical_state_t critical;

    if (token == 0 || CommandManagement_IsRemoteSource(source) == 0U)
    {
        return 0U;
    }
    *token   = (command_refresh_token_t){0};
    critical = PlatformCritical_Enter();
    if (source_commands[source].enable == 0U || slot_generation[source] != expected_slot_generation
        || accepted_command_id[source] == 0UL)
    {
        PlatformCritical_Exit(critical);
        return 0U;
    }
    *token = (command_refresh_token_t){
        .source              = source,
        .slot_generation     = slot_generation[source],
        .accepted_command_id = accepted_command_id[source],
        .revoke_generation   = motion_revoke_generation,
        .mode_generation     = mode_generation,
        .linear_x            = source_commands[source].linear_x,
        .angular_z           = source_commands[source].angular_z,
        .mode                = accepted_mode[source],
    };
    PlatformCritical_Exit(critical);
    return 1U;
}

uint8_t CommandManagement_RefreshAccepted(const command_refresh_token_t *token, uint32_t now_ms)
{
    platform_critical_state_t critical;
    command_velocity_t       *command;
    command_source_t          source;

    if (token == 0 || CommandManagement_IsRemoteSource(token->source) == 0U)
    {
        return 0U;
    }
    source   = token->source;
    critical = PlatformCritical_Enter();
    command  = &source_commands[source];
    if (motion_allowed == 0U || (permitted_source_mask & COMMAND_SOURCE_BIT(source)) == 0U
        || (rearm_required_mask & COMMAND_SOURCE_BIT(source)) != 0U || command->enable == 0U
        || slot_generation[source] != token->slot_generation
        || accepted_command_id[source] != token->accepted_command_id
        || motion_revoke_generation != token->revoke_generation || mode_generation != token->mode_generation
        || accepted_mode[source] != token->mode || command->source != source || command->linear_x != token->linear_x
        || command->angular_z != token->angular_z
        || (uint32_t)(now_ms - command->timestamp_ms) > CommandManagement_Timeout(source)
        || (uint32_t)(now_ms - accepted_at_ms[source]) > command_config.remote_max_lifetime_ms)
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
                                                              uint32_t                  expected_generation,
                                                              uint8_t                   identity_mode)
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
    if ((permitted_source_mask & COMMAND_SOURCE_BIT(sanitized.source)) == 0U)
    {
        PlatformCritical_Exit(critical);
        return (command_apply_result_t){.outcome         = COMMAND_OUTCOME_SOURCE_NOT_ALLOWED,
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
    if ((rearm_required_mask & COMMAND_SOURCE_BIT(sanitized.source)) != 0U)
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
    if (sanitized.linear_x == 0.0f)
    {
        sanitized.linear_x = 0.0f;
    }
    if (sanitized.angular_z == 0.0f)
    {
        sanitized.angular_z = 0.0f;
    }
    if ((params.wheel_radius_m <= 0.0f || params.track_width_m <= 0.0f)
        && CommandManagement_Abs(sanitized.angular_z) > 0.0001f)
    {
        CommandManagement_ClearSource(sanitized.source);
        return (command_apply_result_t){.outcome         = COMMAND_OUTCOME_INVALID,
                                        .source_cleared  = 1U,
                                        .slot_generation = slot_generation[sanitized.source]};
    }

    critical = PlatformCritical_Enter();
    if (motion_allowed == 0U || (permitted_source_mask & COMMAND_SOURCE_BIT(sanitized.source)) == 0U
        || (rearm_required_mask & COMMAND_SOURCE_BIT(sanitized.source)) != 0U
        || (enforce_generation != 0U && motion_revoke_generation != expected_generation))
    {
        PlatformCritical_Exit(critical);
        command_outcome_t outcome = (motion_allowed == 0U) ? COMMAND_OUTCOME_GATE_CLOSED
                                    : ((permitted_source_mask & COMMAND_SOURCE_BIT(sanitized.source)) == 0U)
                                        ? COMMAND_OUTCOME_SOURCE_NOT_ALLOWED
                                        : COMMAND_OUTCOME_GENERATION_CONFLICT;
        return (command_apply_result_t){.outcome = outcome, .slot_generation = slot_generation[sanitized.source]};
    }
    source_commands[sanitized.source] = sanitized;
    command_generation++;
    slot_generation[sanitized.source]++;
    accepted_command_id[sanitized.source]++;
    if (accepted_command_id[sanitized.source] == 0UL)
    {
        accepted_command_id[sanitized.source] = 1UL;
    }
    accepted_at_ms[sanitized.source] = sanitized.timestamp_ms;
    accepted_mode[sanitized.source]  = identity_mode;
    command_apply_result_t result    = {.outcome         = COMMAND_OUTCOME_ACTIVE_ACCEPTED,
                                        .slot_generation = slot_generation[sanitized.source]};
    PlatformCritical_Exit(critical);
    return result;
}

command_apply_result_t CommandManagement_Apply(const command_velocity_t *command)
{
    return CommandManagement_ApplyInternal(command, 0U, 0UL, 0U);
}

command_apply_result_t CommandManagement_ApplyForGeneration(const command_velocity_t *command,
                                                            uint32_t                  expected_generation)
{
    return CommandManagement_ApplyInternal(command, 1U, expected_generation, 0U);
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

command_apply_result_t CommandManagement_ApplyIntent(const command_intent_t *intent)
{
    command_apply_result_t result = {COMMAND_OUTCOME_INVALID, 0U, 0UL};

    if (intent == 0 || CommandManagement_SourceValid(intent->source) == 0U)
    {
        return result;
    }
    if (intent->kind == COMMAND_INTENT_ACTIVE || intent->kind == COMMAND_INTENT_NEUTRAL)
    {
        command_velocity_t command = {
            .linear_x     = (intent->kind == COMMAND_INTENT_NEUTRAL) ? 0.0f : intent->linear_x,
            .angular_z    = (intent->kind == COMMAND_INTENT_NEUTRAL) ? 0.0f : intent->angular_z,
            .enable       = 1U,
            .source       = intent->source,
            .timestamp_ms = intent->sample_time_ms,
        };

        return CommandManagement_ApplyInternal(&command, 1U, intent->expected_revoke_generation, intent->mode);
    }
    if (intent->kind == COMMAND_INTENT_RELEASE)
    {
        CommandManagement_ClearSource(intent->source);
        result.outcome        = COMMAND_OUTCOME_RELEASE_ACCEPTED;
        result.source_cleared = 1U;
        return result;
    }
    if (intent->kind == COMMAND_INTENT_REARM)
    {
        return CommandManagement_QualifyRearm(intent->source);
    }
    if (intent->kind == COMMAND_INTENT_REMOTE_DISABLE
        && (intent->source == COMMAND_SOURCE_HOST || intent->source == COMMAND_SOURCE_ESP12F))
    {
        return CommandManagement_QualifyRearm(intent->source);
    }
    return result;
}

uint8_t CommandManagement_GetActive(command_velocity_t *command, uint32_t now_ms)
{
    return CommandManagement_GetActiveSnapshot(now_ms, command, 0);
}

uint8_t
CommandManagement_GetActiveSnapshot(uint32_t now_ms, command_velocity_t *command, command_management_status_t *status)
{
    platform_critical_state_t critical;
    uint8_t                   found = 0U;

    if (command != 0)
    {
        *command = (command_velocity_t){0};
    }
    critical = PlatformCritical_Enter();
    if (motion_allowed != 0U)
    {
        for (uint8_t index = 0U; index < (uint8_t)(sizeof(source_priority) / sizeof(source_priority[0])); ++index)
        {
            command_velocity_t snapshot = source_commands[source_priority[index]];

            if (snapshot.enable != 0U && CommandManagement_SourceValid(snapshot.source) != 0U
                && (permitted_source_mask & COMMAND_SOURCE_BIT(snapshot.source)) != 0U
                && (uint32_t)(now_ms - snapshot.timestamp_ms) <= CommandManagement_Timeout(snapshot.source))
            {
                if (command != 0)
                {
                    *command = snapshot;
                }
                found = 1U;
                break;
            }
        }
    }
    if (status != 0)
    {
        status->active_source            = (found != 0U && command != 0) ? command->source : COMMAND_SOURCE_NONE;
        status->motion_allowed           = motion_allowed;
        status->permitted_source_mask    = permitted_source_mask;
        status->rearm_required_mask      = rearm_required_mask;
        status->motion_revoke_generation = motion_revoke_generation;
        status->mode_generation          = mode_generation;
        status->generation               = command_generation;
    }
    PlatformCritical_Exit(critical);
    return found;
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

uint8_t CommandManagement_ApplySourcePolicy(uint8_t  permitted_mask,
                                            uint8_t  rearm_mask,
                                            uint8_t  qualify_mask,
                                            uint32_t next_mode_generation)
{
    platform_critical_state_t critical;
    uint8_t                   disallowed_mask;

    if ((permitted_mask & (uint8_t)~COMMAND_ALL_SOURCE_MASK) != 0U
        || (rearm_mask & (uint8_t)~COMMAND_ALL_SOURCE_MASK) != 0U || (qualify_mask & (uint8_t)~permitted_mask) != 0U)
    {
        return 0U;
    }
    critical = PlatformCritical_Enter();
    if ((int32_t)(next_mode_generation - mode_generation) < 0)
    {
        PlatformCritical_Exit(critical);
        return 0U;
    }
    disallowed_mask = (uint8_t)(COMMAND_ALL_SOURCE_MASK & (uint8_t)~permitted_mask);
    for (uint8_t index = 1U; index < COMMAND_SOURCE_COUNT; ++index)
    {
        uint8_t bit = COMMAND_SOURCE_BIT(index);

        if ((disallowed_mask & bit) != 0U || (rearm_mask & bit) != 0U)
        {
            source_commands[index] = (command_velocity_t){0};
            slot_generation[index]++;
        }
    }
    permitted_source_mask = permitted_mask;
    rearm_required_mask |= (uint8_t)(disallowed_mask | rearm_mask);
    rearm_required_mask &= (uint8_t)~qualify_mask;
    mode_generation = next_mode_generation;
    motion_revoke_generation++;
    command_generation++;
    PlatformCritical_Exit(critical);
    return 1U;
}

void CommandManagement_SetMotionGate(uint8_t allowed, uint32_t decision_generation)
{
    uint8_t                   next_allowed = (allowed != 0U) ? 1U : 0U;
    int32_t                   generation_delta;
    platform_critical_state_t critical = PlatformCritical_Enter();

    generation_delta = (int32_t)(decision_generation - motion_gate_generation);

    if (generation_delta < 0 || (generation_delta == 0 && next_allowed == motion_allowed))
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
                && (permitted_source_mask & COMMAND_SOURCE_BIT(snapshot.source)) != 0U
                && (uint32_t)(now_ms - snapshot.timestamp_ms) <= CommandManagement_Timeout(snapshot.source))
            {
                status->active_source = snapshot.source;
                break;
            }
        }
    }
    status->motion_allowed           = motion_allowed;
    status->permitted_source_mask    = permitted_source_mask;
    status->rearm_required_mask      = rearm_required_mask;
    status->motion_revoke_generation = motion_revoke_generation;
    status->mode_generation          = mode_generation;
    status->generation               = command_generation;
    PlatformCritical_Exit(critical);
    return status->generation;
}
