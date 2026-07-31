#include "motor_driver_snapshot_adapter.h"

#include "motor_driver.h"
#include "motor_hardware_layout.h"
#include "platform_critical.h"

static uint32_t motor_fact_generation;

_Static_assert(APP_MOTOR_DRIVER_COUNT == MOTOR_ID_COUNT, "App motor snapshot must cover every hardware motor");

static void AppMotorDriverAdapter_AdvanceGeneration(void)
{
    platform_critical_state_t critical = PlatformCritical_Enter();

    motor_fact_generation++;
    if (motor_fact_generation == 0UL)
    {
        motor_fact_generation = 1UL;
    }
    PlatformCritical_Exit(critical);
}

void AppMotorDriverAdapter_UpdateFaults(void)
{
    MotorDriver_UpdateFaults();
    AppMotorDriverAdapter_AdvanceGeneration();
}

uint32_t AppMotorDriverAdapter_GetSnapshot(app_motor_driver_snapshot_t *snapshot)
{
    motor_driver_state_t      state;
    platform_critical_state_t critical;

    if (snapshot == 0)
    {
        return 0UL;
    }
    MotorDriver_GetState(&state);
    *snapshot = (app_motor_driver_snapshot_t){0};
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        snapshot->requested_pwm[index] = state.requested_pwm[index];
        snapshot->applied_pwm[index]   = state.applied_pwm[index];
        snapshot->effective_pwm[index] = state.effective_pwm[index];
        snapshot->fault_active[index]  = state.fault_active[index];
        if (MotorHardwareLayout_MotorEnabled((motor_id_t)index) != 0U)
        {
            snapshot->enabled_mask |= (uint8_t)(1U << index);
        }
    }
    snapshot->tim1_break_latched = state.tim1_break_latched;
    critical                     = PlatformCritical_Enter();
    snapshot->generation         = motor_fact_generation;
    PlatformCritical_Exit(critical);
    return snapshot->generation;
}

uint8_t AppMotorDriverAdapter_ClearBreakLatch(void)
{
    uint8_t cleared = MotorDriver_ClearBreakLatch();

    if (cleared != 0U)
    {
        AppMotorDriverAdapter_AdvanceGeneration();
    }
    return cleared;
}
