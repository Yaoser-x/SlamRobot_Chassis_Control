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
    if (fake_begin_allowed == 0U || fake_lock != 0U)
    {
        return 0U;
    }
    fake_lock = 1U;
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
    require_int(fake_lock != 0U, "mode changes while Safety maintenance gate is closed");
    fake_mode = mode;
    return 1U;
}

void MotionControl_CancelTestMode(void)
{
    require_int(fake_lock != 0U && fake_mode == CONTROL_MODE_MAINTENANCE,
                "maintenance mode owns command sources before canceling test mode");
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

int main(void)
{
    test_success_holds_lock_until_end();
    test_busy_does_not_take_or_release_lock();
    test_manual_mode_rejects_maintenance();
    test_active_control_step_defers_maintenance();
    test_invalid_encoder_releases_lock();
    test_any_pwm_stage_rejects_maintenance();
    (void)printf("PASS: App motion maintenance host tests\n");
    return 0;
}
