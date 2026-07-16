#include "control_service.h"

#include "bsp_config.h"
#include "command_management_service.h"
#include "control_config.h"
#include "param_service.h"
#include "platform_time.h"
#include "safety_management_service.h"

static void ControlService_EnsureCommandInitialized(void)
{
    if (CommandManagement_IsInitialized() == 0U)
    {
        const command_management_config_t config = {
            CONTROL_TIMEOUT_UPPER_MS,
            CONTROL_TIMEOUT_PS2_MS,
            CONTROL_TIMEOUT_ESP12F_MS,
            CONTROL_TIMEOUT_LINE_MS,
            CONTROL_TIMEOUT_DEBUG_MS,
        };
        (void)CommandManagement_Init(&config);
    }
}

static void ControlService_EnsureSafetyInitialized(void)
{
    if (SafetyManagement_IsInitialized() == 0U)
    {
        const safety_management_config_t config = {
            .battery_low_warn_v           = BATTERY_LOW_WARN_V,
            .battery_low_clear_v          = BATTERY_LOW_CLEAR_V,
            .battery_critical_v           = BATTERY_CRITICAL_V,
            .battery_recover_v            = BATTERY_RECOVER_V,
            .battery_critical_debounce_ms = BATTERY_CRITICAL_DEBOUNCE_MS,
            .battery_recover_debounce_ms  = BATTERY_RECOVER_DEBOUNCE_MS,
            .update_period_ms             = CHASSIS_ADC_PERIOD_MS,
            .overcurrent_startup_blank_ms = MOTOR_OVERCURRENT_STARTUP_BLANK_MS,
            .overcurrent_startup_rearm_ms = MOTOR_OVERCURRENT_STARTUP_REARM_MS,
            .battery_low_monitor_enabled  = BATTERY_LOW_MONITOR_ENABLED,
            .overcurrent_fault_enabled    = MOTOR_ADC_OVERCURRENT_FAULT_ENABLED,
            .current_observe_a            = {1.5f, 1.5f, 1.5f, 1.5f},
            .current_fault_a              = {2.5f, 2.5f, 2.5f, 2.5f},
            .current_fault_debounce_ms    = 100U,
        };

        ControlService_EnsureCommandInitialized();
        (void)SafetyManagement_Init(&config);
    }
}

static void ControlService_EnsureParametersInitialized(void)
{
    param_model_t params;

    (void)ParamService_GetSnapshot(&params);
}

void ControlService_Init(void)
{
    if (CommandManagement_IsInitialized() == 0U)
    {
        ControlService_EnsureCommandInitialized();
    }
    else
    {
        CommandManagement_ClearAll();
    }
}

control_command_result_t ControlService_SetCommand(const chassis_cmd_t *cmd)
{
    ControlService_EnsureCommandInitialized();
    ControlService_EnsureParametersInitialized();
    return CommandManagement_Set(cmd);
}

control_command_result_t ControlService_SetCommandForGeneration(const chassis_cmd_t *cmd, uint32_t expected_generation)
{
    ControlService_EnsureCommandInitialized();
    ControlService_EnsureParametersInitialized();
    return CommandManagement_SetForGeneration(cmd, expected_generation);
}

void ControlService_SetEmergencyStop(uint8_t enabled)
{
    ControlService_EnsureSafetyInitialized();
    SafetyManagement_SetEmergencyStop(enabled);
}

void ControlService_SetFaultStop(uint8_t enabled)
{
    ControlService_EnsureSafetyInitialized();
    SafetyManagement_SetFaultStop(enabled);
}

uint8_t ControlService_BeginMaintenance(void)
{
    ControlService_EnsureSafetyInitialized();
    return SafetyManagement_BeginMaintenance();
}

void ControlService_EndMaintenance(void)
{
    ControlService_EnsureSafetyInitialized();
    SafetyManagement_EndMaintenance();
}

uint8_t ControlService_IsMaintenanceLocked(void)
{
    ControlService_EnsureSafetyInitialized();
    return SafetyManagement_IsMaintenanceLocked();
}

uint32_t ControlService_GetMotionRevokeGeneration(void)
{
    ControlService_EnsureCommandInitialized();
    return CommandManagement_GetMotionRevokeGeneration();
}

void ControlService_ClearCommand(void)
{
    ControlService_EnsureCommandInitialized();
    CommandManagement_ClearAll();
}

void ControlService_ClearSource(uint8_t source)
{
    ControlService_EnsureCommandInitialized();
    CommandManagement_ClearSource((command_source_t)source);
}

uint8_t ControlService_GetCommand(chassis_cmd_t *cmd, uint32_t now_ms)
{
    ControlService_EnsureCommandInitialized();
    return CommandManagement_GetActive(cmd, now_ms);
}

uint8_t ControlService_IsEmergencyStop(void)
{
    ControlService_EnsureSafetyInitialized();
    return SafetyManagement_IsEmergencyStop();
}

uint8_t ControlService_IsFaultStop(void)
{
    ControlService_EnsureSafetyInitialized();
    return SafetyManagement_IsFaultStop();
}

uint8_t ControlService_GetActiveSource(void)
{
    ControlService_EnsureCommandInitialized();
    return (uint8_t)CommandManagement_GetActiveSource(PlatformTime_TaskNowMs());
}
