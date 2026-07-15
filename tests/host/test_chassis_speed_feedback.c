#include <assert.h>
#include <stdio.h>

#include "chassis_feedback_guard.h"
#include "chassis_layout.h"
#include "chassis_speed_loop.h"
#include "control_config.h"

static void TestSpeedLoopFreezePreservesFeedforward(void)
{
    chassis_speed_loop_t        loop;
    chassis_speed_loop_result_t result;

    ChassisSpeedLoop_Init(&loop);
    result =
        ChassisSpeedLoop_StepMotor(&loop, MOTOR_ID_M1, 0.0f, 0.0f, 0U, 0.01f, 0, 321, MOTOR_DRIVER_PHASE_RAMP_DOWN);
    assert(result.permille == 321);
    assert(result.pid_active == 0U);
    assert(result.error_mps == 0.0f);
}

static void TestSpeedLoopStopClearsOutput(void)
{
    chassis_speed_loop_t        loop;
    chassis_speed_loop_result_t result;

    ChassisSpeedLoop_Init(&loop);
    result = ChassisSpeedLoop_StepMotor(&loop, MOTOR_ID_M1, 0.0f, 0.0f, 1U, 0.01f, 0, 321, MOTOR_DRIVER_PHASE_RUN);
    assert(result.permille == 0);
    assert(result.pid_active == 0U);
}

static void TestFeedbackLossCounter(void)
{
    chassis_feedback_guard_t   guard;
    chassis_service_snapshot_t snapshot = {0};
    motor_id_t                 motor    = MOTOR_ID_COUNT;

    ChassisFeedbackGuard_Init(&guard);
    for (uint32_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        if (ChassisLayout_MotorEnabled((motor_id_t)index) != 0U)
        {
            motor = (motor_id_t)index;
            break;
        }
    }
    assert((uint32_t)motor < MOTOR_ID_COUNT);
    assert(ChassisLayout_MotorEnabled(motor) != 0U);
    for (uint32_t count = 1U; count < CHASSIS_PID_FEEDBACK_LOSS_COUNT; ++count)
    {
        assert(ChassisFeedbackGuard_CheckUsable(&guard, &snapshot, motor, 0.5f, 0.0f, 1U) != 0U);
        assert(snapshot.motor_feedback_lost[motor] == 0U);
    }
    assert(ChassisFeedbackGuard_CheckUsable(&guard, &snapshot, motor, 0.5f, 0.0f, 1U) == 0U);
    assert(snapshot.motor_feedback_lost[motor] != 0U);

    assert(ChassisFeedbackGuard_CheckUsable(&guard, &snapshot, motor, 0.5f, 0.1f, 1U) != 0U);
    assert(snapshot.motor_feedback_lost[motor] == 0U);
}

int main(void)
{
    TestSpeedLoopFreezePreservesFeedforward();
    TestSpeedLoopStopClearsOutput();
    TestFeedbackLossCounter();
    puts("chassis speed/feedback tests passed");
    return 0;
}
