#include "imu_calibration_orchestrator.h"

#include "motion_control_maintenance.h"
#include "motion_control_service.h"
#include "parameter_management_service.h"
#include "platform_critical.h"
#include "power_management_service.h"
#include "state_estimation_composition.h"
#include "state_estimation_maintenance.h"

#define APP_IMU_AUTOSAVE_RETRY_MS     1000U
#define APP_IMU_AUTOSAVE_MAX_ATTEMPTS 3U

typedef struct
{
    uint32_t save_next_ms;
    uint8_t  first_save_needed;
    uint8_t  save_pending;
    uint8_t  save_attempts;
    uint8_t  persist_imu_calibration;
    uint8_t  persist_current_zero;
} app_imu_calibration_persistence_t;

static app_imu_calibration_persistence_t persistence;

void ImuCalibrationOrchestrator_Init(uint8_t persist_imu_calibration, uint8_t persist_current_zero)
{
    platform_critical_state_t critical = PlatformCritical_Enter();

    persistence = (app_imu_calibration_persistence_t){
        .first_save_needed       = 1U,
        .persist_imu_calibration = (persist_imu_calibration != 0U) ? 1U : 0U,
        .persist_current_zero    = (persist_current_zero != 0U) ? 1U : 0U,
    };
    PlatformCritical_Exit(critical);
    ParameterManagement_SetCurrentZeroPersistence(persist_current_zero);
    StateEstimation_InitCalibrationCoordinator();
}

void ImuCalibrationOrchestrator_ProcessSample(uint32_t now_ms)
{
    motion_control_status_t                     motion;
    state_estimation_calibration_motion_facts_t motion_facts = {0};
    uint8_t                                     completed;
    platform_critical_state_t                   critical;

    (void)MotionControl_GetStatus(&motion);
    for (uint8_t index = 0U; index < 4U; ++index)
    {
        motion_facts.output_permille[index] = motion.motor_effective_output_permille[index];
    }
    motion_facts.enabled_mask = motion.motor_enabled_mask;
    completed                 = StateEstimation_ServiceCalibrationCoordinator(now_ms, &motion_facts);
    if (completed == 0U)
    {
        return;
    }

    critical = PlatformCritical_Enter();
    if (persistence.persist_imu_calibration != 0U && persistence.first_save_needed != 0U)
    {
        persistence.first_save_needed = 0U;
        persistence.save_pending      = 1U;
        persistence.save_attempts     = 0U;
        persistence.save_next_ms      = now_ms;
    }
    PlatformCritical_Exit(critical);
}

void ImuCalibrationOrchestrator_ProcessPersistence(uint32_t now_ms)
{
    imu_calibration_t         calibration;
    param_model_t             params;
    power_management_status_t current;
    uint32_t                  save_next_ms;
    uint8_t                   save_pending;
    uint8_t                   persist_current_zero;
    uint8_t                   saved;
    platform_critical_state_t critical;

    critical             = PlatformCritical_Enter();
    save_pending         = persistence.save_pending;
    save_next_ms         = persistence.save_next_ms;
    persist_current_zero = persistence.persist_current_zero;
    PlatformCritical_Exit(critical);
    if (save_pending == 0U || (int32_t)(now_ms - save_next_ms) < 0)
    {
        return;
    }
    if (MotionControl_BeginMaintenance() != MOTION_CONTROL_MAINTENANCE_OK)
    {
        return;
    }
    if (StateEstimation_GetImuCalibration(&calibration) != (uint8_t)STATE_ESTIMATION_RESULT_OK)
    {
        critical                 = PlatformCritical_Enter();
        persistence.save_next_ms = now_ms + APP_IMU_AUTOSAVE_RETRY_MS;
        PlatformCritical_Exit(critical);
        MotionControl_EndMaintenance();
        return;
    }

    (void)ParameterManagement_GetSnapshot(&params);
    (void)PowerManagement_GetStatus(&current);
    if (persist_current_zero != 0U && current.current_zero_valid != 0U)
    {
        for (uint8_t index = 0U; index < 4U; ++index)
        {
            params.current_zero_raw[index] = current.current_zero_raw[index];
        }
        params.current_zero_valid = 1U;
        (void)ParameterManagement_Set(&params);
    }
    else if (persist_current_zero == 0U)
    {
        ParameterManagement_SetCurrentZeroPersistence(0U);
    }
    ParameterManagement_SetImuCalibration(&calibration);
    saved = ParameterManagement_Save();

    critical = PlatformCritical_Enter();
    persistence.save_attempts++;
    if (saved != 0U)
    {
        persistence.save_pending  = 0U;
        persistence.save_attempts = 0U;
    }
    else if (persistence.save_attempts >= APP_IMU_AUTOSAVE_MAX_ATTEMPTS)
    {
        persistence.save_pending = 0U;
    }
    else
    {
        persistence.save_next_ms = now_ms + APP_IMU_AUTOSAVE_RETRY_MS;
    }
    PlatformCritical_Exit(critical);
    MotionControl_EndMaintenance();
}
