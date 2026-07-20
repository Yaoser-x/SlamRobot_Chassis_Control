#include "command_management_service.h"

#include "parameter_management_service.h"

#include <stdio.h>
#include <stdlib.h>

uint32_t ParameterManagement_GetSnapshot(param_model_t *params)
{
    if (params == NULL)
    {
        return 0UL;
    }
    *params                 = (param_model_t){0};
    params->max_linear_mps  = 0.5f;
    params->max_angular_rps = 10.0f;
    params->wheel_radius_m  = 0.035f;
    params->track_width_m   = 0.176f;
    return 1UL;
}

static void require_int(int condition, const char *message)
{
    if (condition == 0)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

static command_velocity_t command(command_source_t source, uint32_t timestamp_ms)
{
    command_velocity_t value = {
        .linear_x     = 0.2f,
        .angular_z    = 0.1f,
        .enable       = 1U,
        .source       = source,
        .timestamp_ms = timestamp_ms,
    };

    return value;
}

static void reset_owner(void)
{
    const command_management_config_t config = {
        .host_timeout_ms   = 200U,
        .ps2_timeout_ms    = 500U,
        .esp12f_timeout_ms = 500U,
        .line_timeout_ms   = 50U,
        .debug_timeout_ms  = 2000U,
    };

    require_int(CommandManagement_Init(&config) != 0U, "command owner initializes");
}

static void test_priority_timeout_and_ordinary_fallback(void)
{
    command_velocity_t active;
    command_velocity_t debug = command(COMMAND_SOURCE_DEBUG, 100U);
    command_velocity_t line  = command(COMMAND_SOURCE_LINE, 100U);
    command_velocity_t ps2   = command(COMMAND_SOURCE_PS2, 100U);
    command_velocity_t host  = command(COMMAND_SOURCE_HOST, 100U);

    reset_owner();
    require_int(CommandManagement_Set(&debug) == COMMAND_RESULT_ACCEPTED, "debug accepted");
    require_int(CommandManagement_Set(&line) == COMMAND_RESULT_ACCEPTED, "line accepted");
    require_int(CommandManagement_Set(&ps2) == COMMAND_RESULT_ACCEPTED, "PS2 accepted");
    require_int(CommandManagement_Set(&host) == COMMAND_RESULT_ACCEPTED, "host accepted");
    require_int(CommandManagement_GetActive(&active, 120U) != 0U && active.source == COMMAND_SOURCE_HOST,
                "host has fixed highest priority");

    CommandManagement_ClearSource(COMMAND_SOURCE_HOST);
    require_int(CommandManagement_GetActive(&active, 120U) != 0U && active.source == COMMAND_SOURCE_PS2,
                "ordinary source removal restores unexpired PS2 command");
    require_int(CommandManagement_GetActive(&active, 601U) != 0U && active.source == COMMAND_SOURCE_DEBUG,
                "source-specific timeouts fall back to unexpired debug command");
}

static void test_safety_gate_revokes_without_old_command_recovery(void)
{
    command_velocity_t active;
    command_velocity_t host = command(COMMAND_SOURCE_HOST, 100U);
    uint32_t           revoke_generation;

    reset_owner();
    require_int(CommandManagement_Set(&host) == COMMAND_RESULT_ACCEPTED, "host accepted before gate close");
    revoke_generation = CommandManagement_GetMotionRevokeGeneration();
    CommandManagement_SetMotionGate(0U, 10U);
    require_int(CommandManagement_GetMotionRevokeGeneration() == revoke_generation + 1UL,
                "gate close increments revoke generation once");
    require_int(CommandManagement_GetActive(&active, 101U) == 0U, "closed gate exposes no command");
    CommandManagement_SetMotionGate(1U, 11U);
    require_int(CommandManagement_GetActive(&active, 102U) == 0U, "gate reopen does not restore old command");
    require_int(CommandManagement_SetForGeneration(&host, revoke_generation) == COMMAND_RESULT_REJECTED,
                "pre-revoke producer generation is rejected");
}

static void test_stale_safety_decision_cannot_reopen_gate(void)
{
    command_velocity_t host = command(COMMAND_SOURCE_HOST, 100U);

    reset_owner();
    CommandManagement_SetMotionGate(0U, 20U);
    CommandManagement_SetMotionGate(1U, 19U);
    require_int(CommandManagement_Set(&host) == COMMAND_RESULT_REJECTED,
                "older safety decision cannot reopen a newer closed gate");
    CommandManagement_SetMotionGate(1U, 21U);
    require_int(CommandManagement_Set(&host) == COMMAND_RESULT_ACCEPTED, "newer safety decision reopens gate");
}

static void test_duplicate_refresh_preserves_generation_and_cannot_revive_timeout(void)
{
    command_management_status_t before;
    command_management_status_t after;
    command_velocity_t          host = command(COMMAND_SOURCE_HOST, 100U);

    reset_owner();
    require_int(CommandManagement_Set(&host) == COMMAND_RESULT_ACCEPTED, "host accepted before refresh");
    (void)CommandManagement_GetStatus(150U, &before);
    require_int(CommandManagement_RefreshSource(COMMAND_SOURCE_HOST, 150U) != 0U, "live lease refresh succeeds");
    (void)CommandManagement_GetStatus(150U, &after);
    require_int(after.generation == before.generation, "refresh does not increment command generation");
    require_int(CommandManagement_RefreshSource(COMMAND_SOURCE_HOST, 351U) == 0U,
                "expired duplicate cannot revive command");
}

int main(void)
{
    test_priority_timeout_and_ordinary_fallback();
    test_safety_gate_revokes_without_old_command_recovery();
    test_stale_safety_decision_cannot_reopen_gate();
    test_duplicate_refresh_preserves_generation_and_cannot_revive_timeout();
    return 0;
}
