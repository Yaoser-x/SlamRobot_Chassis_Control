#include "motion_maintenance_orchestrator.h"

#include "control_mode_coordinator.h"
#include "motor_driver_snapshot_adapter.h"
#include "motor_types.h"
#include "motion_control_maintenance.h"
#include "motion_control_service.h"
#include "robot_config.h"
#include "safety_workflow_coordinator.h"
#include "state_estimation_service.h"

#include <stdio.h>
#include <stdlib.h>

static uint8_t                         fake_lock;
static uint8_t                         fake_begin_allowed;
static uint32_t                        cancel_count;
static uint32_t                        emergency_stop_count;
static uint32_t                        end_count;
static uint32_t                        begin_count;
static uint8_t                         move_after_acquire;
static state_estimation_wheel_status_t fake_encoder;
static app_motor_driver_snapshot_t     fake_motor;
static uint8_t                         fake_control_step_active;
static control_mode_t                  fake_mode;
static robot_config_t                  fake_config;

static void require_int(int condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

uint8_t AppSafetyWorkflow_BeginMaintenance(void)
{
    begin_count++;
    if (fake_begin_allowed == 0U || fake_lock != 0U)
    {
        return 0U;
    }
    fake_lock = 1U;
    if (move_after_acquire != 0U)
    {
        fake_encoder.speed_mps[MOTOR_ID_M2] = 0.1f;
    }
    return 1U;
}

void AppSafetyWorkflow_EndMaintenance(void)
{
    fake_lock = 0U;
    end_count++;
}

uint32_t ControlModeCoordinator_GetSnapshot(control_mode_snapshot_t *snapshot)
{
    *snapshot      = (control_mode_snapshot_t){0};
    snapshot->mode = fake_mode;
    return 1UL;
}

uint8_t ControlModeCoordinator_Request(control_mode_t mode)
{
    fake_mode = mode;
    return 1U;
}

void MotionControl_CancelTestMode(void)
{
    cancel_count++;
}

void MotionControl_EmergencyStop(void)
{
    require_int(fake_lock != 0U, "maintenance locks before chassis stop");
    emergency_stop_count++;
}

uint8_t MotionControl_IsStepActive(void)
{
    return fake_control_step_active;
}

uint32_t StateEstimation_GetWheel(state_estimation_wheel_status_t *state)
{
    *state = fake_encoder;
    return 1U;
}

uint32_t AppMotorDriverAdapter_GetSnapshot(app_motor_driver_snapshot_t *state)
{
    *state              = (app_motor_driver_snapshot_t){0};
    state->enabled_mask = (uint8_t)((1U << MOTOR_ID_COUNT) - 1U);
    state->generation   = 1UL;
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        state->requested_pwm[index] = fake_motor.requested_pwm[index];
        state->applied_pwm[index]   = fake_motor.applied_pwm[index];
        state->effective_pwm[index] = fake_motor.effective_pwm[index];
    }
    return state->generation;
}

const robot_config_t *RobotConfig_GetDefault(void)
{
    return &fake_config;
}

static void reset_fake(void)
{
    fake_lock                                    = 0U;
    fake_begin_allowed                           = 1U;
    cancel_count                                 = 0U;
    emergency_stop_count                         = 0U;
    end_count                                    = 0U;
    begin_count                                  = 0U;
    move_after_acquire                           = 0U;
    fake_encoder                                 = (state_estimation_wheel_status_t){0};
    fake_motor                                   = (app_motor_driver_snapshot_t){0};
    fake_control_step_active                     = 0U;
    fake_mode                                    = CONTROL_MODE_AUTO;
    fake_config                                  = (robot_config_t){0};
    fake_config.motion.maintenance_max_speed_mps = 0.02f;
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        fake_encoder.speed_valid[index] = 1U;
    }
}

static void test_success_holds_lock_until_end(void)
{
    reset_fake();
    require_int(AppMotionMaintenance_Begin() == APP_MOTION_MAINTENANCE_OK, "stationary maintenance begins");
    require_int(fake_lock != 0U && fake_mode == CONTROL_MODE_MAINTENANCE, "successful maintenance holds ownership");
    require_int(cancel_count == 1U && emergency_stop_count == 1U, "maintenance stops through the Motion output owner");
    AppMotionMaintenance_End();
    require_int(fake_lock == 0U && end_count == 1U && fake_mode == CONTROL_MODE_AUTO,
                "maintenance end restores mode before releasing Safety gate");
}

static void test_busy_does_not_take_or_release_lock(void)
{
    reset_fake();
    fake_begin_allowed = 0U;
    require_int(AppMotionMaintenance_Begin() == APP_MOTION_MAINTENANCE_BUSY, "busy maintenance is distinguishable");
    require_int(cancel_count == 0U && end_count == 0U, "busy path changes no ownership");
}

static void test_manual_mode_rejects_maintenance(void)
{
    reset_fake();
    fake_mode = CONTROL_MODE_MANUAL;
    require_int(AppMotionMaintenance_Begin() == APP_MOTION_MAINTENANCE_BUSY,
                "PS2 takeover prevents maintenance acquisition");
    require_int(fake_lock == 0U && end_count == 0U, "manual rejection does not touch Safety ownership");
}

static void test_active_control_step_defers_maintenance(void)
{
    reset_fake();
    fake_control_step_active = 1U;
    require_int(AppMotionMaintenance_Begin() == APP_MOTION_MAINTENANCE_BUSY,
                "active motor control step defers maintenance");
    require_int(fake_lock == 0U && end_count == 1U && fake_mode == CONTROL_MODE_AUTO,
                "deferred maintenance restores mode before releasing lock");
    require_int(cancel_count == 0U && emergency_stop_count == 0U,
                "deferred maintenance does not race the active output step");
}

static void test_invalid_encoder_releases_lock(void)
{
    reset_fake();
    fake_encoder.speed_valid[MOTOR_ID_M2] = 0U;
    require_int(AppMotionMaintenance_Begin() == APP_MOTION_MAINTENANCE_NOT_STATIONARY,
                "invalid enabled encoder rejects maintenance");
    require_int(fake_lock == 0U && end_count == 1U && fake_mode == CONTROL_MODE_AUTO,
                "rejected maintenance restores mode and releases lock");
}

static void test_any_pwm_stage_rejects_maintenance(void)
{
    reset_fake();
    fake_motor.requested_pwm[MOTOR_ID_M3] = 1;
    require_int(AppMotionMaintenance_Begin() == APP_MOTION_MAINTENANCE_NOT_STATIONARY,
                "requested pwm rejects maintenance");

    reset_fake();
    fake_motor.applied_pwm[MOTOR_ID_M3] = 1;
    require_int(AppMotionMaintenance_Begin() == APP_MOTION_MAINTENANCE_NOT_STATIONARY,
                "applied pwm rejects maintenance");

    reset_fake();
    fake_motor.effective_pwm[MOTOR_ID_M3] = 1;
    require_int(AppMotionMaintenance_Begin() == APP_MOTION_MAINTENANCE_NOT_STATIONARY,
                "effective pwm rejects maintenance");
}

static void test_diagnostic_motion_switches_mode_without_safety_lock(void)
{
    reset_fake();
    require_int(AppMotionMaintenance_BeginDiagnosticMotion() == APP_MOTION_MAINTENANCE_OK,
                "stationary diagnostic motion begins");
    require_int(fake_lock == 0U && fake_mode == CONTROL_MODE_MAINTENANCE,
                "diagnostic motion selects DEBUG mode without the stop-only Safety lock");
    require_int(cancel_count == 1U && emergency_stop_count == 0U,
                "diagnostic motion cancels raw test mode without forcing an emergency stop");
    require_int(AppMotionMaintenance_Begin() == APP_MOTION_MAINTENANCE_BUSY,
                "configuration maintenance cannot overlap diagnostic motion");
    AppMotionMaintenance_EndDiagnosticMotion();
    require_int(fake_mode == CONTROL_MODE_AUTO && fake_lock == 0U,
                "diagnostic motion restores the previous product mode");
}

static void test_diagnostic_motion_requires_stationary_chassis(void)
{
    reset_fake();
    fake_encoder.speed_mps[MOTOR_ID_M2] = 0.1f;
    require_int(AppMotionMaintenance_BeginDiagnosticMotion() == APP_MOTION_MAINTENANCE_NOT_STATIONARY,
                "moving chassis rejects diagnostic motion");
    require_int(fake_mode == CONTROL_MODE_AUTO && fake_lock == 0U,
                "diagnostic rejection leaves mode and Safety ownership unchanged");
}

static void test_stationary_try_rejects_motion_without_revoking_it(void)
{
    reset_fake();
    fake_encoder.speed_mps[MOTOR_ID_M2] = 0.1f;
    require_int(AppMotionMaintenance_TryBeginStationary() == APP_MOTION_MAINTENANCE_NOT_STATIONARY,
                "background maintenance waits for an already stationary chassis");
    require_int(begin_count == 0UL && end_count == 0UL && cancel_count == 0UL && emergency_stop_count == 0UL
                    && fake_mode == CONTROL_MODE_AUTO,
                "failed stationary preflight changes no gate, source mode, test lease or motor output");
    reset_fake();
    fake_control_step_active = 1U;
    require_int(AppMotionMaintenance_TryBeginStationary() == APP_MOTION_MAINTENANCE_BUSY,
                "background maintenance defers active Motor cycle before acquiring Safety");
    require_int(begin_count == 0UL && end_count == 0UL && emergency_stop_count == 0UL,
                "active cycle preflight has no side effects");
}

static void test_stationary_try_rechecks_after_acquisition(void)
{
    reset_fake();
    move_after_acquire = 1U;
    require_int(AppMotionMaintenance_TryBeginStationary() == APP_MOTION_MAINTENANCE_NOT_STATIONARY,
                "background maintenance rejects facts that changed after acquisition");
    require_int(fake_lock == 0U && fake_mode == CONTROL_MODE_AUTO && end_count == 1UL && emergency_stop_count == 0UL,
                "failed second proof releases ownership before any Flash or output operation");
    reset_fake();
    require_int(AppMotionMaintenance_TryBeginStationary() == APP_MOTION_MAINTENANCE_OK,
                "stationary background maintenance acquires ownership");
    require_int(fake_lock != 0U && fake_mode == CONTROL_MODE_MAINTENANCE && emergency_stop_count == 1UL,
                "successful background acquisition resets the output owner while locked");
    AppMotionMaintenance_End();
}

int main(void)
{
    test_success_holds_lock_until_end();
    test_busy_does_not_take_or_release_lock();
    test_manual_mode_rejects_maintenance();
    test_active_control_step_defers_maintenance();
    test_invalid_encoder_releases_lock();
    test_any_pwm_stage_rejects_maintenance();
    test_diagnostic_motion_switches_mode_without_safety_lock();
    test_diagnostic_motion_requires_stationary_chassis();
    test_stationary_try_rejects_motion_without_revoking_it();
    test_stationary_try_rechecks_after_acquisition();
    (void)printf("PASS: App motion maintenance host tests\n");
    return 0;
}
