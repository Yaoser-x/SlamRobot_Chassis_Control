#include "chassis_runtime_coordinator.h"

#include "command_management_service.h"
#include "motion_control_service.h"
#include "parameter_management_service.h"
#include "power_management_service.h"
#include "power_on_self_test_service.h"
#include "safety_management_service.h"
#include "state_estimation_service.h"

#define CHASSIS_RUNTIME_IMU_FRESH_MS 100U
#define CHASSIS_RUNTIME_IMU_BLOCKING_QUALITY                                                                           \
    (STATE_ESTIMATION_IMU_QUALITY_SPI_ERROR | STATE_ESTIMATION_IMU_QUALITY_INIT_FAILED                                 \
     | STATE_ESTIMATION_IMU_QUALITY_TIMESTAMP_ERROR | STATE_ESTIMATION_IMU_QUALITY_PROFILE_MISMATCH)

static uint32_t motor_completion_generation;
static uint32_t safety_completion_generation;
static uint32_t watchdog_motor_generation;
static uint8_t  communication_initialized;
static uint8_t  boot_qualified;

static uint8_t ChassisRuntimeCoordinator_OutputsZero(const motion_control_status_t *motion)
{
    for (uint8_t index = 0U; index < MOTION_CONTROL_MOTOR_COUNT; ++index)
    {
        if (motion->motor_effective_output_permille[index] != 0 || motion->motor_output_permille[index] != 0)
        {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t ChassisRuntimeCoordinator_NormalFactsValid(uint32_t now_ms)
{
    parameter_management_status_t params;
    power_management_status_t     power;
    state_estimation_status_t     state;
    motion_control_status_t       motion;
    safety_management_status_t    safety;
    power_on_self_test_result_t   post;

    (void)ParameterManagement_GetStatus(&params);
    (void)PowerManagement_GetStatus(&power);
    (void)StateEstimation_GetStatus(now_ms, &state);
    (void)MotionControl_GetStatus(&motion);
    (void)SafetyManagement_GetStatus(&safety);
    PowerOnSelfTest_GetResult(&post);
    if (params.initialized == 0U || params.generation == 0UL || communication_initialized == 0U
        || motor_completion_generation == 0UL || post.done == 0U || post.error_flags != 0UL
        || power.current_zero_valid == 0U || power.current_control_valid == 0U || state.wheel.speed_valid_all == 0U
        || state.imu.online == 0U || state.imu.chip_id == 0U || state.imu_fresh == 0U
        || (uint32_t)(now_ms - state.imu.last_update_ms) > CHASSIS_RUNTIME_IMU_FRESH_MS
        || (state.imu.quality_flags & CHASSIS_RUNTIME_IMU_BLOCKING_QUALITY) != 0UL
        || (safety.error_flags & (SYSTEM_ERROR_DRV_FAULT | SYSTEM_ERROR_TIM_BREAK)) != 0UL)
    {
        return 0U;
    }
    return (boot_qualified != 0U || ChassisRuntimeCoordinator_OutputsZero(&motion) != 0U) ? 1U : 0U;
}

static uint8_t ChassisRuntimeCoordinator_DiagnosticFactsValid(void)
{
    power_management_status_t  power;
    safety_management_status_t safety;

    (void)PowerManagement_GetStatus(&power);
    (void)SafetyManagement_GetStatus(&safety);
    return (power.current_zero_valid != 0U && power.current_control_valid != 0U
            && (safety.error_flags & (SYSTEM_ERROR_DRV_FAULT | SYSTEM_ERROR_TIM_BREAK)) == 0UL
            && safety.emergency_stop == 0U && safety.fault_stop == 0U)
               ? 1U
               : 0U;
}

void ChassisRuntimeCoordinator_Init(void)
{
    motor_completion_generation  = 0UL;
    safety_completion_generation = 0UL;
    watchdog_motor_generation    = 0UL;
    communication_initialized    = 1U;
    boot_qualified               = 0U;
    SafetyManagement_ApplyRuntimePermit(0U, SAFETY_STATE_BOOT_SAFE);
    SafetyManagement_ApplyDiagnosticPermit(0U);
}

void ChassisRuntimeCoordinator_RunMotorCycle(uint32_t now_ms)
{
    StateEstimation_UpdateWheel(now_ms);
    PowerManagement_UpdateStationary();
    MotionControl_Step(now_ms);
    motor_completion_generation++;
}

void ChassisRuntimeCoordinator_RunSafetyCycle(uint32_t now_ms)
{
    param_model_t          params;
    uint8_t                permit;
    safety_runtime_state_t runtime_state;

    (void)ParameterManagement_GetSnapshot(&params);
    SafetyManagement_SetCurrentThresholds(params.current_observe_a,
                                          params.current_fault_a,
                                          params.current_fault_debounce_ms);
    SafetyManagement_Update();
    PostService_UpdateRuntime(now_ms);
    SafetyManagement_ApplyDiagnosticPermit(ChassisRuntimeCoordinator_DiagnosticFactsValid());
    permit = ChassisRuntimeCoordinator_NormalFactsValid(now_ms);
    if (permit != 0U)
    {
        boot_qualified = 1U;
    }
    runtime_state = (permit == 0U)
                        ? ((boot_qualified != 0U) ? SAFETY_STATE_STANDBY : SAFETY_STATE_BOOT_SAFE)
                        : ((CommandManagement_GetActiveSource(now_ms) == COMMAND_SOURCE_NONE) ? SAFETY_STATE_STANDBY
                                                                                              : SAFETY_STATE_ACTIVE);
    SafetyManagement_ApplyRuntimePermit(permit, runtime_state);
    safety_completion_generation++;
}

uint8_t ChassisRuntimeCoordinator_ShouldFeedWatchdog(void)
{
    if (safety_completion_generation == 0UL || motor_completion_generation == watchdog_motor_generation)
    {
        return 0U;
    }
    watchdog_motor_generation = motor_completion_generation;
    return 1U;
}

uint32_t ChassisRuntimeCoordinator_GetMotorCompletionGeneration(void)
{
    return motor_completion_generation;
}

uint32_t ChassisRuntimeCoordinator_GetSafetyCompletionGeneration(void)
{
    return safety_completion_generation;
}
