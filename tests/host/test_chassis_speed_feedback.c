#include <assert.h>
#include <stdio.h>

#include "wheel_feedback_monitor.h"
#include "motor_hardware_layout.h"
#include "wheel_speed_control_loop.h"
#include "control_config.h"

static const motion_control_config_t motion_config = {
    .pid_correction_limit        = CHASSIS_PID_CORRECTION_LIMIT,
    .pid_stop_epsilon_mps        = CHASSIS_PID_STOP_EPSILON_MPS,
    .pid_direction_epsilon_mps   = CHASSIS_PID_DIRECTION_EPSILON_MPS,
    .pid_feedback_min_target_mps = CHASSIS_PID_FEEDBACK_MIN_TARGET_MPS,
    .pid_feedback_min_speed_mps  = CHASSIS_PID_FEEDBACK_MIN_SPEED_MPS,
    .encoder_feedback_timeout_ms = CHASSIS_ENCODER_FEEDBACK_TIMEOUT_MS,
    .pid_feedback_loss_count     = CHASSIS_PID_FEEDBACK_LOSS_COUNT,
};

static void TestSpeedLoopFreezePreservesFeedforward(void)
{
    wheel_speed_control_loop_t        loop;
    wheel_speed_control_loop_result_t result;

    WheelSpeedControlLoop_Init(&loop, &motion_config);
    result = WheelSpeedControlLoop_StepMotor(&loop,
                                             MOTOR_ID_M1,
                                             0.0f,
                                             0.0f,
                                             0U,
                                             0.01f,
                                             0,
                                             321,
                                             MOTOR_DRIVER_PHASE_RAMP_DOWN);
    assert(result.permille == 321);
    assert(result.pid_active == 0U);
    assert(result.error_mps == 0.0f);
}

static void TestSpeedLoopStopClearsOutput(void)
{
    wheel_speed_control_loop_t        loop;
    wheel_speed_control_loop_result_t result;

    WheelSpeedControlLoop_Init(&loop, &motion_config);
    result = WheelSpeedControlLoop_StepMotor(&loop, MOTOR_ID_M1, 0.0f, 0.0f, 1U, 0.01f, 0, 321, MOTOR_DRIVER_PHASE_RUN);
    assert(result.permille == 0);
    assert(result.pid_active == 0U);
}

static void TestFeedbackLossCounter(void)
{
    wheel_feedback_monitor_t guard;
    motion_control_status_t  snapshot = {0};
    motor_id_t               motor    = MOTOR_ID_COUNT;

    WheelFeedbackMonitor_Init(&guard, &motion_config);
    for (uint32_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        if (MotorHardwareLayout_MotorEnabled((motor_id_t)index) != 0U)
        {
            motor = (motor_id_t)index;
            break;
        }
    }
    assert((uint32_t)motor < MOTOR_ID_COUNT);
    assert(MotorHardwareLayout_MotorEnabled(motor) != 0U);
    for (uint32_t count = 1U; count < CHASSIS_PID_FEEDBACK_LOSS_COUNT; ++count)
    {
        assert(WheelFeedbackMonitor_CheckUsable(&guard, &snapshot, motor, 0.5f, 0.0f, 1U) != 0U);
        assert(snapshot.motor_feedback_lost[motor] == 0U);
    }
    assert(WheelFeedbackMonitor_CheckUsable(&guard, &snapshot, motor, 0.5f, 0.0f, 1U) == 0U);
    assert(snapshot.motor_feedback_lost[motor] != 0U);

    assert(WheelFeedbackMonitor_CheckUsable(&guard, &snapshot, motor, 0.5f, 0.1f, 1U) != 0U);
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
