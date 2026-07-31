#include "control_mode_coordinator.h"

#include "command_management_service.h"
#include "platform_critical.h"

static control_mode_config_t   mode_config;
static control_mode_snapshot_t mode_snapshot;
static uint8_t                 stick_takeover_latched;
static uint8_t                 takeover_sample_count;
static uint8_t                 manual_neutral_timing_active;
static uint8_t                 capability_source_mask;

static uint8_t ControlModeCoordinator_ModeMask(control_mode_t mode)
{
    switch (mode)
    {
        case CONTROL_MODE_MANUAL:
            return COMMAND_SOURCE_MASK_PS2;
        case CONTROL_MODE_AUTO:
            return COMMAND_SOURCE_MASK_REMOTE;
        case CONTROL_MODE_LINE:
            return COMMAND_SOURCE_MASK_LINE;
        case CONTROL_MODE_MAINTENANCE:
            return COMMAND_SOURCE_MASK_DEBUG;
        case CONTROL_MODE_DISABLED:
        default:
            return 0U;
    }
}

static uint8_t
ControlModeCoordinator_Transition(control_mode_t next_mode, uint8_t qualify_mask, uint8_t takeover_active)
{
    control_mode_snapshot_t next = mode_snapshot;
    uint8_t effective_mask       = (uint8_t)(ControlModeCoordinator_ModeMask(next_mode) & capability_source_mask);

    next.mode = next_mode;
    next.generation++;
    next.takeover_active         = takeover_active;
    next.manual_neutral_since_ms = 0U;
    manual_neutral_timing_active = 0U;
    if (CommandManagement_ApplySourcePolicy(effective_mask,
                                            COMMAND_ALL_SOURCE_MASK,
                                            (uint8_t)(qualify_mask & effective_mask),
                                            next.generation)
        == 0U)
    {
        return 0U;
    }
    mode_snapshot = next;
    return 1U;
}

uint8_t ControlModeCoordinator_ValidateConfig(const control_mode_config_t *config)
{
    return ControlModeConfig_Validate(config);
}

uint8_t ControlModeCoordinator_Init(const control_mode_config_t *config)
{
    if (ControlModeCoordinator_ValidateConfig(config) == 0U)
    {
        return 0U;
    }
    mode_config                  = *config;
    mode_snapshot                = (control_mode_snapshot_t){0};
    mode_snapshot.mode           = CONTROL_MODE_AUTO;
    mode_snapshot.recovery_mode  = CONTROL_MODE_AUTO;
    mode_snapshot.generation     = 1UL;
    stick_takeover_latched       = 0U;
    takeover_sample_count        = 0U;
    manual_neutral_timing_active = 0U;
    capability_source_mask       = 0U;
    return CommandManagement_ApplySourcePolicy(0U, COMMAND_ALL_SOURCE_MASK, 0U, mode_snapshot.generation);
}

uint8_t ControlModeCoordinator_ApplyCapabilityMask(uint8_t capability_mask)
{
    platform_critical_state_t critical;
    uint8_t                   effective_mask;
    uint8_t                   result = 1U;

    if ((capability_mask & (uint8_t)~COMMAND_ALL_SOURCE_MASK) != 0U)
    {
        return 0U;
    }
    critical = PlatformCritical_Enter();
    if (capability_source_mask != capability_mask)
    {
        effective_mask = (uint8_t)(ControlModeCoordinator_ModeMask(mode_snapshot.mode) & capability_mask);
        result =
            CommandManagement_ApplySourcePolicy(effective_mask, COMMAND_ALL_SOURCE_MASK, 0U, mode_snapshot.generation);
        if (result != 0U)
        {
            capability_source_mask = capability_mask;
        }
    }
    PlatformCritical_Exit(critical);
    return result;
}

static uint8_t ControlModeCoordinator_RequestLocked(control_mode_t mode)
{
    if (mode >= CONTROL_MODE_COUNT || mode == CONTROL_MODE_MANUAL || mode_snapshot.mode == CONTROL_MODE_MANUAL)
    {
        return 0U;
    }
    if (mode_snapshot.mode == mode)
    {
        return 1U;
    }
    mode_snapshot.recovery_mode = mode;
    return ControlModeCoordinator_Transition(mode, 0U, 0U);
}

uint8_t ControlModeCoordinator_Request(control_mode_t mode)
{
    platform_critical_state_t critical = PlatformCritical_Enter();
    uint8_t                   result   = ControlModeCoordinator_RequestLocked(mode);

    PlatformCritical_Exit(critical);
    return result;
}

static control_mode_event_t ControlModeCoordinator_EnterManual(void)
{
    control_mode_t previous = mode_snapshot.mode;

    mode_snapshot.recovery_mode =
        (previous == CONTROL_MODE_AUTO || previous == CONTROL_MODE_LINE) ? previous : CONTROL_MODE_DISABLED;
    if (ControlModeCoordinator_Transition(CONTROL_MODE_MANUAL, COMMAND_SOURCE_MASK_PS2, 1U) == 0U)
    {
        return CONTROL_MODE_EVENT_NONE;
    }
    stick_takeover_latched = 1U;
    takeover_sample_count  = 0U;
    return CONTROL_MODE_EVENT_ENTERED_MANUAL;
}

static control_mode_event_t ControlModeCoordinator_Restore(void)
{
    control_mode_t recovery = mode_snapshot.recovery_mode;

    if (recovery != CONTROL_MODE_AUTO && recovery != CONTROL_MODE_LINE)
    {
        recovery = CONTROL_MODE_DISABLED;
    }
    if (ControlModeCoordinator_Transition(recovery, 0U, 0U) == 0U)
    {
        return CONTROL_MODE_EVENT_NONE;
    }
    stick_takeover_latched = 0U;
    takeover_sample_count  = 0U;
    return (recovery == CONTROL_MODE_LINE) ? CONTROL_MODE_EVENT_RESTORED_LINE : CONTROL_MODE_EVENT_RESTORED_AUTO;
}

static control_mode_event_t ControlModeCoordinator_UpdatePs2Locked(const teleoperation_status_t *status,
                                                                   uint32_t                      now_ms)
{
    uint8_t edge_takeover;
    uint8_t neutral;

    if (status == 0)
    {
        return CONTROL_MODE_EVENT_NONE;
    }
    if (mode_snapshot.mode == CONTROL_MODE_MANUAL)
    {
        if (status->online == 0U)
        {
            mode_snapshot.recovery_mode = CONTROL_MODE_DISABLED;
            (void)ControlModeCoordinator_Transition(CONTROL_MODE_DISABLED, 0U, 0U);
            stick_takeover_latched       = 0U;
            takeover_sample_count        = 0U;
            manual_neutral_timing_active = 0U;
            return CONTROL_MODE_EVENT_PS2_DISCONNECTED;
        }
        if (status->sample_valid == 0U)
        {
            mode_snapshot.manual_neutral_since_ms = 0U;
            manual_neutral_timing_active          = 0U;
            return CONTROL_MODE_EVENT_NONE;
        }
        neutral = (status->stick_max_normalized <= mode_config.takeover_exit_threshold && status->dpad_active == 0U
                   && status->macro_active == 0U && status->macro_edge == 0U)
                      ? 1U
                      : 0U;
        if (neutral == 0U)
        {
            mode_snapshot.manual_neutral_since_ms = 0U;
            manual_neutral_timing_active          = 0U;
            return CONTROL_MODE_EVENT_NONE;
        }
        if (manual_neutral_timing_active == 0U)
        {
            mode_snapshot.manual_neutral_since_ms = now_ms;
            manual_neutral_timing_active          = 1U;
            return CONTROL_MODE_EVENT_NONE;
        }
        if ((uint32_t)(now_ms - mode_snapshot.manual_neutral_since_ms) >= mode_config.manual_neutral_restore_ms)
        {
            return ControlModeCoordinator_Restore();
        }
        return CONTROL_MODE_EVENT_NONE;
    }

    if (status->sample_valid == 0U || status->online == 0U
        || (mode_snapshot.mode != CONTROL_MODE_AUTO && mode_snapshot.mode != CONTROL_MODE_LINE))
    {
        takeover_sample_count = 0U;
        return CONTROL_MODE_EVENT_NONE;
    }
    if (status->stick_max_normalized >= mode_config.takeover_enter_threshold)
    {
        stick_takeover_latched = 1U;
    }
    else if (status->stick_max_normalized <= mode_config.takeover_exit_threshold)
    {
        stick_takeover_latched = 0U;
    }
    edge_takeover = (status->dpad_active != 0U || status->macro_edge != 0U) ? 1U : 0U;
    if (edge_takeover != 0U)
    {
        return ControlModeCoordinator_EnterManual();
    }
    if (stick_takeover_latched == 0U)
    {
        takeover_sample_count = 0U;
        return CONTROL_MODE_EVENT_NONE;
    }
    if (takeover_sample_count < mode_config.takeover_confirm_samples)
    {
        takeover_sample_count++;
    }
    return (takeover_sample_count >= mode_config.takeover_confirm_samples) ? ControlModeCoordinator_EnterManual()
                                                                           : CONTROL_MODE_EVENT_NONE;
}

control_mode_event_t ControlModeCoordinator_UpdatePs2(const teleoperation_status_t *status, uint32_t now_ms)
{
    platform_critical_state_t critical = PlatformCritical_Enter();
    control_mode_event_t      event    = ControlModeCoordinator_UpdatePs2Locked(status, now_ms);

    PlatformCritical_Exit(critical);
    return event;
}

uint32_t ControlModeCoordinator_GetSnapshot(control_mode_snapshot_t *snapshot)
{
    platform_critical_state_t critical;

    if (snapshot == 0)
    {
        return 0UL;
    }
    critical  = PlatformCritical_Enter();
    *snapshot = mode_snapshot;
    PlatformCritical_Exit(critical);
    return snapshot->generation;
}
