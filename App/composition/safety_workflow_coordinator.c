#include "safety_workflow_coordinator.h"

#include "command_management_service.h"
#include "motor_driver_snapshot_adapter.h"
#include "platform_time.h"
#include "safety_management_service.h"
#include "state_estimation_service.h"

void AppSafetyWorkflow_SynchronizeCommandGate(void)
{
    safety_management_status_t safety;

    (void)SafetyManagement_GetStatus(&safety);
    CommandManagement_SetMotionGate(safety.motion_allowed, safety.gate_decision_generation);
}

void AppSafetyWorkflow_SetEmergencyStop(uint8_t enabled)
{
    SafetyManagement_SetEmergencyStop(enabled);
    AppSafetyWorkflow_SynchronizeCommandGate();
}

void AppSafetyWorkflow_SetFaultStop(uint8_t enabled)
{
    SafetyManagement_SetFaultStop(enabled);
    AppSafetyWorkflow_SynchronizeCommandGate();
}

void AppSafetyWorkflow_LatchEncoderFeedbackFault(void)
{
    SafetyManagement_LatchEncoderFeedbackFault();
    AppSafetyWorkflow_SynchronizeCommandGate();
}

safety_clear_result_t AppSafetyWorkflow_ClearLatchedFaults(uint32_t mask)
{
    safety_clear_input_t        input = {0};
    safety_management_status_t  safety;
    app_motor_driver_snapshot_t motor;
    safety_clear_result_t       result;
    uint32_t                    now_ms = PlatformTime_TaskNowMs();

    (void)SafetyManagement_GetStatus(&safety);
    input.wheel.generation               = StateEstimation_GetWheel(&input.wheel.value);
    input.wheel.sample_time_ms           = input.wheel.value.last_update_ms;
    input.wheel.validity                 = (input.wheel.generation != 0UL) ? 1UL : 0UL;
    input.wheel.quality                  = input.wheel.value.current_anomaly_mask;
    input.motor.generation               = AppMotorDriverAdapter_GetSnapshot(&motor);
    input.motor.sample_time_ms           = now_ms;
    input.motor.validity                 = (input.motor.generation != 0UL) ? 1UL : 0UL;
    input.motor.quality                  = motor.tim1_break_latched;
    input.motor.value.tim1_break_latched = motor.tim1_break_latched;
    input.motor.value.enabled_mask       = motor.enabled_mask;
    for (uint8_t index = 0U; index < APP_MOTOR_DRIVER_COUNT; ++index)
    {
        input.motor.value.requested_pwm[index] = motor.requested_pwm[index];
        input.motor.value.applied_pwm[index]   = motor.applied_pwm[index];
        input.motor.value.effective_pwm[index] = motor.effective_pwm[index];
        input.motor.value.fault_active[index]  = motor.fault_active[index];
    }
    if ((mask & SYSTEM_ERROR_TIM_BREAK) != 0UL && safety.emergency_stop == 0U)
    {
        input.tim_break_clear_succeeded      = AppMotorDriverAdapter_ClearBreakLatch();
        input.motor.generation               = AppMotorDriverAdapter_GetSnapshot(&motor);
        input.motor.value.tim1_break_latched = motor.tim1_break_latched;
    }
    result = SafetyManagement_ClearLatchedFaults(mask, &input);

    AppSafetyWorkflow_SynchronizeCommandGate();
    return result;
}

uint8_t AppSafetyWorkflow_BeginMaintenance(void)
{
    uint8_t acquired = SafetyManagement_BeginMaintenance();

    AppSafetyWorkflow_SynchronizeCommandGate();
    return acquired;
}

void AppSafetyWorkflow_EndMaintenance(void)
{
    SafetyManagement_EndMaintenance();
    AppSafetyWorkflow_SynchronizeCommandGate();
}
