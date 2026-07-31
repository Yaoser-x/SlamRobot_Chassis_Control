#include "command_management_service.h"

#include "parameter_management_service.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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
        .host_timeout_ms        = 200U,
        .ps2_timeout_ms         = 500U,
        .esp12f_timeout_ms      = 500U,
        .line_timeout_ms        = 50U,
        .debug_timeout_ms       = 2000U,
        .remote_max_lifetime_ms = 2000U,
    };

    require_int(CommandManagement_Init(&config) != 0U, "command owner initializes");
    require_int(CommandManagement_IsMotionGateOpen() == 0U, "command owner starts fail-closed");
    CommandManagement_SetMotionGate(1U, 1U);
    require_int(CommandManagement_QualifyRearm(COMMAND_SOURCE_PS2).outcome == COMMAND_OUTCOME_RELEASE_ACCEPTED,
                "PS2 neutral qualification rearms PS2 only");
    require_int(CommandManagement_QualifyRearm(COMMAND_SOURCE_LINE).outcome == COMMAND_OUTCOME_RELEASE_ACCEPTED,
                "line off qualification rearms line only");
    require_int(CommandManagement_QualifyRearm(COMMAND_SOURCE_DEBUG).outcome == COMMAND_OUTCOME_RELEASE_ACCEPTED,
                "debug stop qualification rearms debug only");
}

static void rearm_host(void)
{
    require_int(CommandManagement_DisableRemoteSource(COMMAND_SOURCE_HOST) == COMMAND_RESULT_ACCEPTED,
                "host disable establishes rearm qualification");
}

static void test_priority_timeout_and_ordinary_fallback(void)
{
    command_velocity_t active;
    command_velocity_t debug = command(COMMAND_SOURCE_DEBUG, 100U);
    command_velocity_t line  = command(COMMAND_SOURCE_LINE, 100U);
    command_velocity_t ps2   = command(COMMAND_SOURCE_PS2, 100U);
    command_velocity_t host  = command(COMMAND_SOURCE_HOST, 100U);

    reset_owner();
    rearm_host();
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
    command_velocity_t          active;
    command_velocity_t          host = command(COMMAND_SOURCE_HOST, 100U);
    command_velocity_t          esp  = command(COMMAND_SOURCE_ESP12F, 100U);
    command_management_status_t status;
    uint32_t                    revoke_generation;

    reset_owner();
    rearm_host();
    require_int(CommandManagement_Set(&host) == COMMAND_RESULT_ACCEPTED, "host accepted before gate close");
    revoke_generation = CommandManagement_GetMotionRevokeGeneration();
    CommandManagement_SetMotionGate(0U, 10U);
    require_int(CommandManagement_GetMotionRevokeGeneration() == revoke_generation + 1UL,
                "gate close increments revoke generation once");
    require_int(CommandManagement_GetActive(&active, 101U) == 0U, "closed gate exposes no command");
    require_int(CommandManagement_DisableRemoteSource(COMMAND_SOURCE_HOST) == COMMAND_RESULT_REJECTED,
                "disable during closed gate clears source but cannot rearm");
    CommandManagement_SetMotionGate(1U, 11U);
    require_int(CommandManagement_GetActive(&active, 102U) == 0U, "gate reopen does not restore old command");
    require_int(CommandManagement_Set(&host) == COMMAND_RESULT_REJECTED,
                "gate reopen still requires a new remote disable");
    rearm_host();
    (void)CommandManagement_GetStatus(102U, &status);
    require_int((status.rearm_required_mask & (1U << COMMAND_SOURCE_HOST)) == 0U,
                "new host disable clears only host rearm latch");
    require_int((status.rearm_required_mask & (1U << COMMAND_SOURCE_ESP12F)) != 0U,
                "ESP remains independently rearm-required");
    require_int(CommandManagement_Set(&esp) == COMMAND_RESULT_REJECTED, "host rearm cannot authorize ESP source");
    require_int(CommandManagement_SetForGeneration(&host, revoke_generation) == COMMAND_RESULT_REJECTED,
                "pre-revoke producer generation is rejected");
}

static void test_structured_result_and_all_source_rearm(void)
{
    command_velocity_t     debug = command(COMMAND_SOURCE_DEBUG, 10U);
    command_apply_result_t result;

    reset_owner();
    result = CommandManagement_Apply(&debug);
    require_int(result.outcome == COMMAND_OUTCOME_ACTIVE_ACCEPTED && result.slot_generation > 0UL,
                "structured active result carries slot generation");
    CommandManagement_SetMotionGate(0U, 10U);
    result = CommandManagement_QualifyRearm(COMMAND_SOURCE_DEBUG);
    require_int(result.outcome == COMMAND_OUTCOME_GATE_CLOSED && result.source_cleared != 0U,
                "release while gate closed clears but does not rearm");
    CommandManagement_SetMotionGate(1U, 11U);
    result = CommandManagement_Apply(&debug);
    require_int(result.outcome == COMMAND_OUTCOME_REARM_REQUIRED, "debug must stop again after gate reopens");
    debug.enable = 0U;
    result       = CommandManagement_Apply(&debug);
    require_int(result.outcome == COMMAND_OUTCOME_RELEASE_ACCEPTED,
                "structured release is accepted while the source requires rearm");
    debug.enable   = 1U;
    debug.linear_x = NAN;
    result         = CommandManagement_Apply(&debug);
    require_int(result.outcome == COMMAND_OUTCOME_INVALID && result.source_cleared != 0U,
                "non-finite command is rejected and clears its source");
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
    require_int(CommandManagement_Set(&host) == COMMAND_RESULT_REJECTED,
                "newer safety decision alone does not rearm remote source");
    rearm_host();
    require_int(CommandManagement_Set(&host) == COMMAND_RESULT_ACCEPTED, "newer safety decision reopens gate");
}

static void test_same_safety_decision_is_idempotent(void)
{
    command_management_status_t before;
    command_management_status_t after;
    command_velocity_t          host = command(COMMAND_SOURCE_HOST, 100U);
    command_apply_result_t      accepted;
    command_refresh_token_t     token;
    uint32_t                    revoke_generation;

    reset_owner();
    rearm_host();
    accepted = CommandManagement_Apply(&host);
    require_int(accepted.outcome == COMMAND_OUTCOME_ACTIVE_ACCEPTED, "host is accepted before repeated gate sync");
    require_int(CommandManagement_GetRefreshToken(COMMAND_SOURCE_HOST, accepted.slot_generation, &token) != 0U,
                "active host command has a refresh token before repeated gate sync");
    (void)CommandManagement_GetStatus(120U, &before);

    CommandManagement_SetMotionGate(1U, 1U);
    (void)CommandManagement_GetStatus(120U, &after);
    require_int(after.generation == before.generation, "same open decision does not mutate command state");
    require_int(CommandManagement_RefreshAccepted(&token, 120U) != 0U,
                "same open decision preserves the accepted slot identity");

    CommandManagement_SetMotionGate(0U, 2U);
    revoke_generation = CommandManagement_GetMotionRevokeGeneration();
    (void)CommandManagement_GetStatus(121U, &before);
    CommandManagement_SetMotionGate(0U, 2U);
    (void)CommandManagement_GetStatus(121U, &after);
    require_int(after.generation == before.generation, "same closed decision does not mutate command state");
    require_int(CommandManagement_GetMotionRevokeGeneration() == revoke_generation,
                "same closed decision does not advance the revoke generation");
}

static void test_duplicate_refresh_preserves_generation_and_cannot_revive_timeout(void)
{
    command_management_status_t before;
    command_management_status_t after;
    command_velocity_t          host = command(COMMAND_SOURCE_HOST, 100U);
    command_apply_result_t      accepted;
    command_refresh_token_t     token;
    command_refresh_token_t     wrong;
    command_intent_t            remote_intent = {
                   .kind                       = COMMAND_INTENT_ACTIVE,
                   .source                     = COMMAND_SOURCE_HOST,
                   .linear_x                   = 0.2f,
                   .angular_z                  = 0.1f,
                   .sample_time_ms             = 100U,
                   .expected_revoke_generation = 0UL,
                   .mode                       = 2U,
    };

    reset_owner();
    rearm_host();
    accepted = CommandManagement_ApplyIntent(&remote_intent);
    require_int(accepted.outcome == COMMAND_OUTCOME_ACTIVE_ACCEPTED, "host accepted before refresh");
    require_int(CommandManagement_GetRefreshToken(COMMAND_SOURCE_HOST, accepted.slot_generation, &token) != 0U,
                "accepted remote command issues a slot-bound refresh token");
    require_int(token.mode == 2U, "refresh token binds the accepted command mode");
    (void)CommandManagement_GetStatus(150U, &before);
    require_int(CommandManagement_RefreshAccepted(&token, 150U) != 0U, "verified duplicate refresh succeeds");
    (void)CommandManagement_GetStatus(150U, &after);
    require_int(after.generation == before.generation, "refresh does not increment command generation");

    wrong = token;
    wrong.accepted_command_id++;
    require_int(CommandManagement_RefreshAccepted(&wrong, 160U) == 0U, "wrong accepted command id is rejected");
    wrong          = token;
    wrong.linear_x = 0.1f;
    require_int(CommandManagement_RefreshAccepted(&wrong, 160U) == 0U, "different normalized payload is rejected");
    wrong = token;
    wrong.revoke_generation++;
    require_int(CommandManagement_RefreshAccepted(&wrong, 160U) == 0U, "revoke generation mismatch is rejected");
    wrong = token;
    wrong.slot_generation++;
    require_int(CommandManagement_RefreshAccepted(&wrong, 160U) == 0U, "slot generation mismatch is rejected");
    wrong = token;
    wrong.mode++;
    require_int(CommandManagement_RefreshAccepted(&wrong, 160U) == 0U, "command mode mismatch is rejected");

    CommandManagement_ClearSource(COMMAND_SOURCE_HOST);
    require_int(CommandManagement_RefreshAccepted(&token, 160U) == 0U, "cleared slot invalidates its token");

    reset_owner();
    rearm_host();
    accepted = CommandManagement_Apply(&host);
    require_int(CommandManagement_GetRefreshToken(COMMAND_SOURCE_HOST, accepted.slot_generation, &token) != 0U,
                "second accepted command issues a token");
    require_int(CommandManagement_RefreshAccepted(&token, 301U) == 0U, "expired per-source lease cannot be revived");

    reset_owner();
    rearm_host();
    accepted = CommandManagement_Apply(&host);
    require_int(CommandManagement_GetRefreshToken(COMMAND_SOURCE_HOST, accepted.slot_generation, &token) != 0U,
                "lifetime test issues a token");
    for (uint32_t now_ms = 200U; now_ms <= 2100U; now_ms += 100U)
    {
        require_int(CommandManagement_RefreshAccepted(&token, now_ms) != 0U,
                    "duplicate remains valid inside original maximum lifetime");
    }
    require_int(CommandManagement_RefreshAccepted(&token, 2101U) == 0U,
                "duplicate cannot extend beyond original maximum lifetime");
}

static void test_active_snapshot_is_coherent(void)
{
    command_velocity_t          active;
    command_velocity_t          host = command(COMMAND_SOURCE_HOST, 100U);
    command_management_status_t status;

    reset_owner();
    rearm_host();
    require_int(CommandManagement_Set(&host) == COMMAND_RESULT_ACCEPTED, "host accepted before snapshot");
    require_int(CommandManagement_GetActiveSnapshot(120U, &active, &status) != 0U,
                "atomic snapshot selects an active command");
    require_int(active.source == COMMAND_SOURCE_HOST && status.active_source == active.source,
                "command value and arbitration status describe the same source");
    require_int(status.generation != 0UL
                    && status.motion_revoke_generation == CommandManagement_GetMotionRevokeGeneration(),
                "snapshot carries coherent command and revoke generations");
}

static void test_normalized_intents_preserve_source_semantics(void)
{
    command_velocity_t active;
    command_intent_t   intent = {
          .kind                       = COMMAND_INTENT_ACTIVE,
          .source                     = COMMAND_SOURCE_DEBUG,
          .linear_x                   = 0.2f,
          .angular_z                  = -0.1f,
          .sample_time_ms             = 100U,
          .producer_generation        = 7UL,
          .expected_revoke_generation = 0UL,
    };
    command_apply_result_t result;

    reset_owner();
    result = CommandManagement_ApplyIntent(&intent);
    require_int(result.outcome == COMMAND_OUTCOME_ACTIVE_ACCEPTED, "active intent is accepted");
    require_int(CommandManagement_GetActive(&active, 100U) != 0U && active.source == COMMAND_SOURCE_DEBUG,
                "active intent reaches the source slot");

    intent.kind           = COMMAND_INTENT_NEUTRAL;
    intent.sample_time_ms = 110U;
    require_int(CommandManagement_ApplyIntent(&intent).outcome == COMMAND_OUTCOME_ACTIVE_ACCEPTED,
                "neutral is a fresh local intent rather than a refresh");
    require_int(CommandManagement_GetActive(&active, 110U) != 0U && active.linear_x == 0.0f && active.angular_z == 0.0f,
                "neutral intent stores explicit zero targets");

    intent.kind = COMMAND_INTENT_RELEASE;
    require_int(CommandManagement_ApplyIntent(&intent).outcome == COMMAND_OUTCOME_RELEASE_ACCEPTED,
                "release intent clears the source");
    require_int(CommandManagement_GetActive(&active, 110U) == 0U, "released source is no longer executable");

    intent.kind   = COMMAND_INTENT_REMOTE_DISABLE;
    intent.source = COMMAND_SOURCE_HOST;
    require_int(CommandManagement_ApplyIntent(&intent).outcome == COMMAND_OUTCOME_RELEASE_ACCEPTED,
                "remote disable keeps its source-specific rearm meaning");
}

static void test_mode_policy_revokes_slots_and_requires_post_manual_rearm(void)
{
    command_velocity_t          active;
    command_velocity_t          host = command(COMMAND_SOURCE_HOST, 100U);
    command_velocity_t          ps2  = command(COMMAND_SOURCE_PS2, 120U);
    command_management_status_t status;
    command_apply_result_t      accepted;
    command_refresh_token_t     token;

    reset_owner();
    rearm_host();
    accepted = CommandManagement_Apply(&host);
    require_int(accepted.outcome == COMMAND_OUTCOME_ACTIVE_ACCEPTED, "host is active before manual takeover");
    require_int(CommandManagement_GetRefreshToken(COMMAND_SOURCE_HOST, accepted.slot_generation, &token) != 0U,
                "pre-takeover host token exists");

    require_int(CommandManagement_ApplySourcePolicy(COMMAND_SOURCE_MASK_PS2,
                                                    COMMAND_ALL_SOURCE_MASK,
                                                    COMMAND_SOURCE_MASK_PS2,
                                                    1U)
                    != 0U,
                "manual policy is applied atomically");
    (void)CommandManagement_GetStatus(120U, &status);
    require_int(status.permitted_source_mask == COMMAND_SOURCE_MASK_PS2 && status.mode_generation == 1U,
                "manual policy publishes PS2-only mask and generation");
    require_int(CommandManagement_GetActive(&active, 120U) == 0U, "manual transition never restores the host slot");
    require_int(CommandManagement_Apply(&host).outcome == COMMAND_OUTCOME_SOURCE_NOT_ALLOWED,
                "active host intent is rejected during manual mode");
    require_int(CommandManagement_QualifyRearm(COMMAND_SOURCE_HOST).outcome == COMMAND_OUTCOME_SOURCE_NOT_ALLOWED,
                "host neutral cannot pre-complete rearm during manual mode");
    require_int(CommandManagement_RefreshAccepted(&token, 130U) == 0U,
                "pre-takeover refresh token is invalid after mode generation changes");
    require_int(CommandManagement_Set(&ps2) == COMMAND_RESULT_ACCEPTED,
                "validated takeover qualifies fresh PS2 intent");

    require_int(CommandManagement_ApplySourcePolicy(COMMAND_SOURCE_MASK_REMOTE, COMMAND_ALL_SOURCE_MASK, 0U, 2U) != 0U,
                "auto recovery restores only the remote source mask");
    require_int(CommandManagement_GetActive(&active, 130U) == 0U, "auto recovery stays stopped before remote rearm");
    require_int(CommandManagement_Apply(&host).outcome == COMMAND_OUTCOME_REARM_REQUIRED,
                "new active host command alone cannot resume after manual mode");
    rearm_host();
    host.timestamp_ms = 140U;
    require_int(CommandManagement_Set(&host) == COMMAND_RESULT_ACCEPTED,
                "fresh disable then active intent resumes auto motion");
    require_int(CommandManagement_ApplySourcePolicy(COMMAND_SOURCE_MASK_LINE,
                                                    COMMAND_ALL_SOURCE_MASK,
                                                    COMMAND_SOURCE_MASK_LINE,
                                                    1U)
                    == 0U,
                "stale mode generation cannot overwrite a newer policy");
}

int main(void)
{
    test_priority_timeout_and_ordinary_fallback();
    test_safety_gate_revokes_without_old_command_recovery();
    test_stale_safety_decision_cannot_reopen_gate();
    test_same_safety_decision_is_idempotent();
    test_duplicate_refresh_preserves_generation_and_cannot_revive_timeout();
    test_structured_result_and_all_source_rearm();
    test_active_snapshot_is_coherent();
    test_normalized_intents_preserve_source_semantics();
    test_mode_policy_revokes_slots_and_requires_post_manual_rearm();
    return 0;
}
