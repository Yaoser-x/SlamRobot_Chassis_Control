#include "chassis_maintenance.h"

#include "chassis_control.h"
#include "chassis_layout.h"
#include "control_manager.h"
#include "encoder_driver.h"
#include "motor_driver.h"

#include <stdio.h>
#include <stdlib.h>

static uint8_t              fake_lock;
static uint8_t              fake_begin_allowed;
static uint32_t             cancel_count;
static uint32_t             emergency_stop_count;
static uint32_t             motor_stop_count;
static uint32_t             end_count;
static encoder_state_t      fake_encoder;
static motor_driver_state_t fake_motor;
static uint8_t              fake_control_step_active;

static void require_int(int condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

uint8_t ControlManager_BeginMaintenance(void)
{
    if (fake_begin_allowed == 0U || fake_lock != 0U)
    {
        return 0U;
    }
    fake_lock = 1U;
    return 1U;
}

void ControlManager_EndMaintenance(void)
{
    fake_lock = 0U;
    end_count++;
}

uint8_t ControlManager_IsMaintenanceLocked(void)
{
    return fake_lock;
}

void ChassisControl_CancelTestMode(void)
{
    require_int(fake_lock != 0U, "maintenance locks before canceling test mode");
    cancel_count++;
}

void ChassisControl_EmergencyStop(void)
{
    require_int(fake_lock != 0U, "maintenance locks before chassis stop");
    emergency_stop_count++;
}

uint8_t ChassisControl_IsStepActive(void)
{
    return fake_control_step_active;
}

void MotorDriver_StopAll(motor_stop_mode_t mode)
{
    (void)mode;
    require_int(fake_lock != 0U, "maintenance locks before motor stop");
    motor_stop_count++;
}

void EncoderDriver_GetState(encoder_state_t *state)
{
    *state = fake_encoder;
}

void MotorDriver_GetState(motor_driver_state_t *state)
{
    *state = fake_motor;
}

uint8_t ChassisLayout_MotorEnabled(motor_id_t motor)
{
    return ((uint32_t)motor < MOTOR_ID_COUNT) ? 1U : 0U;
}

static void reset_fake(void)
{
    fake_lock                = 0U;
    fake_begin_allowed       = 1U;
    cancel_count             = 0U;
    emergency_stop_count     = 0U;
    motor_stop_count         = 0U;
    end_count                = 0U;
    fake_encoder             = (encoder_state_t){0};
    fake_motor               = (motor_driver_state_t){0};
    fake_control_step_active = 0U;
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        fake_encoder.speed_valid[i] = 1U;
    }
}

static void test_success_holds_lock_until_end(void)
{
    reset_fake();
    require_int(ChassisMaintenance_Begin() == CHASSIS_MAINTENANCE_OK, "stationary maintenance begins");
    require_int(fake_lock != 0U, "successful maintenance holds lock");
    require_int(cancel_count == 1U && emergency_stop_count == 1U && motor_stop_count == 1U,
                "maintenance cancels every output path");
    ChassisMaintenance_End();
    require_int(fake_lock == 0U && end_count == 1U, "maintenance end releases lock once");
}

static void test_busy_does_not_take_or_release_lock(void)
{
    reset_fake();
    fake_begin_allowed = 0U;
    require_int(ChassisMaintenance_Begin() == CHASSIS_MAINTENANCE_BUSY, "busy maintenance is distinguishable");
    require_int(cancel_count == 0U && end_count == 0U, "busy path changes no ownership");
}

static void test_active_control_step_defers_maintenance(void)
{
    reset_fake();
    fake_control_step_active = 1U;
    require_int(ChassisMaintenance_Begin() == CHASSIS_MAINTENANCE_BUSY, "active motor control step defers maintenance");
    require_int(fake_lock == 0U && end_count == 1U, "deferred maintenance releases acquired lock");
    require_int(cancel_count == 0U && emergency_stop_count == 0U && motor_stop_count == 0U,
                "deferred maintenance does not race the active output step");
}

static void test_invalid_encoder_releases_lock(void)
{
    reset_fake();
    fake_encoder.speed_valid[MOTOR_ID_M2] = 0U;
    require_int(ChassisMaintenance_Begin() == CHASSIS_MAINTENANCE_NOT_STATIONARY,
                "invalid enabled encoder rejects maintenance");
    require_int(fake_lock == 0U && end_count == 1U, "rejected maintenance releases lock");
}

static void test_any_pwm_stage_rejects_maintenance(void)
{
    reset_fake();
    fake_motor.requested_pwm[MOTOR_ID_M3] = 1;
    require_int(ChassisMaintenance_Begin() == CHASSIS_MAINTENANCE_NOT_STATIONARY, "requested pwm rejects maintenance");

    reset_fake();
    fake_motor.applied_pwm[MOTOR_ID_M3] = 1;
    require_int(ChassisMaintenance_Begin() == CHASSIS_MAINTENANCE_NOT_STATIONARY, "applied pwm rejects maintenance");

    reset_fake();
    fake_motor.effective_pwm[MOTOR_ID_M3] = 1;
    require_int(ChassisMaintenance_Begin() == CHASSIS_MAINTENANCE_NOT_STATIONARY, "effective pwm rejects maintenance");
}

int main(void)
{
    test_success_holds_lock_until_end();
    test_busy_does_not_take_or_release_lock();
    test_active_control_step_defers_maintenance();
    test_invalid_encoder_releases_lock();
    test_any_pwm_stage_rejects_maintenance();
    (void)printf("PASS: chassis maintenance host tests\n");
    return 0;
}
