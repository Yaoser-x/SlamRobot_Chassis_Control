#include "control_mode_coordinator.h"

#include "command_management_service.h"
#include "platform_critical.h"

#include <assert.h>
#include <stdio.h>

static uint8_t  applied_permitted_mask;
static uint8_t  applied_rearm_mask;
static uint8_t  applied_qualify_mask;
static uint32_t applied_mode_generation;
static uint32_t policy_apply_count;

platform_critical_state_t PlatformCritical_Enter(void)
{
    return 0U;
}

void PlatformCritical_Exit(platform_critical_state_t state)
{
    (void)state;
}

uint8_t CommandManagement_ApplySourcePolicy(uint8_t  permitted_mask,
                                            uint8_t  rearm_mask,
                                            uint8_t  qualify_mask,
                                            uint32_t mode_generation)
{
    applied_permitted_mask  = permitted_mask;
    applied_rearm_mask      = rearm_mask;
    applied_qualify_mask    = qualify_mask;
    applied_mode_generation = mode_generation;
    policy_apply_count++;
    return 1U;
}

static control_mode_config_t TestConfig(void)
{
    return (control_mode_config_t){
        .takeover_enter_threshold  = 0.15f,
        .takeover_exit_threshold   = 0.10f,
        .manual_neutral_restore_ms = 2000U,
        .takeover_confirm_samples  = 3U,
    };
}

static teleoperation_status_t ValidPs2(float magnitude)
{
    return (teleoperation_status_t){
        .online               = 1U,
        .sample_valid         = 1U,
        .stick_max_normalized = magnitude,
    };
}

static void InitReady(const control_mode_config_t *config)
{
    assert(ControlModeCoordinator_Init(config) != 0U);
    assert(applied_permitted_mask == 0U);
    assert(applied_rearm_mask == COMMAND_ALL_SOURCE_MASK && applied_qualify_mask == 0U);
    assert(ControlModeCoordinator_ApplyCapabilityMask(COMMAND_ALL_SOURCE_MASK) != 0U);
    assert(applied_permitted_mask == COMMAND_SOURCE_MASK_REMOTE);
}

static void test_stick_hysteresis_and_auto_rearm(void)
{
    control_mode_config_t   config = TestConfig();
    control_mode_snapshot_t snapshot;
    teleoperation_status_t  ps2 = ValidPs2(0.16f);

    InitReady(&config);
    assert(ControlModeCoordinator_UpdatePs2(&ps2, 100U) == CONTROL_MODE_EVENT_NONE);
    ps2.stick_max_normalized = 0.12f;
    assert(ControlModeCoordinator_UpdatePs2(&ps2, 120U) == CONTROL_MODE_EVENT_NONE);
    assert(ControlModeCoordinator_UpdatePs2(&ps2, 140U) == CONTROL_MODE_EVENT_ENTERED_MANUAL);
    assert(applied_permitted_mask == COMMAND_SOURCE_MASK_PS2);
    assert(applied_qualify_mask == COMMAND_SOURCE_MASK_PS2);
    (void)ControlModeCoordinator_GetSnapshot(&snapshot);
    assert(snapshot.mode == CONTROL_MODE_MANUAL && snapshot.recovery_mode == CONTROL_MODE_AUTO);

    ps2.stick_max_normalized = 0.0f;
    assert(ControlModeCoordinator_UpdatePs2(&ps2, 1000U) == CONTROL_MODE_EVENT_NONE);
    assert(ControlModeCoordinator_UpdatePs2(&ps2, 2999U) == CONTROL_MODE_EVENT_NONE);
    assert(ControlModeCoordinator_UpdatePs2(&ps2, 3000U) == CONTROL_MODE_EVENT_RESTORED_AUTO);
    (void)ControlModeCoordinator_GetSnapshot(&snapshot);
    assert(snapshot.mode == CONTROL_MODE_AUTO);
    assert(applied_permitted_mask == COMMAND_SOURCE_MASK_REMOTE && applied_qualify_mask == 0U);
}

static void test_edges_disconnect_and_line_recovery(void)
{
    control_mode_config_t   config = TestConfig();
    control_mode_snapshot_t snapshot;
    teleoperation_status_t  ps2 = ValidPs2(0.0f);

    InitReady(&config);
    assert(ControlModeCoordinator_Request(CONTROL_MODE_LINE) != 0U);
    assert(applied_permitted_mask == COMMAND_SOURCE_MASK_LINE);
    assert(applied_qualify_mask == 0U);

    ps2.dpad_active = 1U;
    assert(ControlModeCoordinator_UpdatePs2(&ps2, 100U) == CONTROL_MODE_EVENT_ENTERED_MANUAL);
    (void)ControlModeCoordinator_GetSnapshot(&snapshot);
    assert(snapshot.recovery_mode == CONTROL_MODE_LINE);
    ps2.dpad_active = 0U;
    assert(ControlModeCoordinator_UpdatePs2(&ps2, 200U) == CONTROL_MODE_EVENT_NONE);
    assert(ControlModeCoordinator_UpdatePs2(&ps2, 2200U) == CONTROL_MODE_EVENT_RESTORED_LINE);
    assert(applied_permitted_mask == COMMAND_SOURCE_MASK_LINE && applied_qualify_mask == 0U);

    ps2.macro_edge = 1U;
    assert(ControlModeCoordinator_UpdatePs2(&ps2, 2300U) == CONTROL_MODE_EVENT_ENTERED_MANUAL);
    ps2.online       = 0U;
    ps2.sample_valid = 0U;
    assert(ControlModeCoordinator_UpdatePs2(&ps2, 2320U) == CONTROL_MODE_EVENT_PS2_DISCONNECTED);
    (void)ControlModeCoordinator_GetSnapshot(&snapshot);
    assert(snapshot.mode == CONTROL_MODE_DISABLED && applied_permitted_mask == 0U);
    assert(applied_mode_generation == snapshot.generation);
}

static void test_manual_neutral_timer_wraparound(void)
{
    control_mode_config_t  config = TestConfig();
    teleoperation_status_t ps2    = ValidPs2(0.16f);

    InitReady(&config);
    assert(ControlModeCoordinator_UpdatePs2(&ps2, 100U) == CONTROL_MODE_EVENT_NONE);
    assert(ControlModeCoordinator_UpdatePs2(&ps2, 120U) == CONTROL_MODE_EVENT_NONE);
    assert(ControlModeCoordinator_UpdatePs2(&ps2, 140U) == CONTROL_MODE_EVENT_ENTERED_MANUAL);
    ps2.stick_max_normalized = 0.0f;
    assert(ControlModeCoordinator_UpdatePs2(&ps2, 0xFFFFFF00UL) == CONTROL_MODE_EVENT_NONE);
    assert(ControlModeCoordinator_UpdatePs2(&ps2, 0x000006CFUL) == CONTROL_MODE_EVENT_NONE);
    assert(ControlModeCoordinator_UpdatePs2(&ps2, 0x000006D0UL) == CONTROL_MODE_EVENT_RESTORED_AUTO);
}

static void test_capability_mask_is_applied_before_source_selection(void)
{
    control_mode_config_t config = TestConfig();
    uint32_t              calls;

    InitReady(&config);
    calls = policy_apply_count;
    assert(ControlModeCoordinator_ApplyCapabilityMask(COMMAND_ALL_SOURCE_MASK) != 0U);
    assert(policy_apply_count == calls);

    assert(ControlModeCoordinator_ApplyCapabilityMask(COMMAND_SOURCE_MASK_PS2 | COMMAND_SOURCE_MASK_LINE) != 0U);
    assert(applied_permitted_mask == 0U);
    assert(applied_rearm_mask == COMMAND_ALL_SOURCE_MASK && applied_qualify_mask == 0U);
    assert(applied_mode_generation == 1U);

    assert(ControlModeCoordinator_ApplyCapabilityMask(COMMAND_ALL_SOURCE_MASK) != 0U);
    assert(applied_permitted_mask == COMMAND_SOURCE_MASK_REMOTE);
    assert(applied_qualify_mask == 0U);
    assert(ControlModeCoordinator_ApplyCapabilityMask(0x80U) == 0U);
    assert(applied_permitted_mask == COMMAND_SOURCE_MASK_REMOTE);
}

int main(void)
{
    control_mode_config_t invalid = TestConfig();

    invalid.takeover_exit_threshold = invalid.takeover_enter_threshold;
    assert(ControlModeCoordinator_ValidateConfig(&invalid) == 0U);
    test_stick_hysteresis_and_auto_rearm();
    test_edges_disconnect_and_line_recovery();
    test_manual_neutral_timer_wraparound();
    test_capability_mask_is_applied_before_source_selection();
    puts("PASS: App control mode coordinator");
    return 0;
}
