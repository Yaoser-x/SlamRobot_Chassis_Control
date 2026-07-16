#include "wheel_encoder_driver.h"

#include "motor_hardware_layout.h"
#include "param_service.h"
#include "tim.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static TIM_TypeDef tim2_instance = {.ARR = 65535U};
static TIM_TypeDef tim3_instance = {.ARR = 65535U};
static TIM_TypeDef tim4_instance = {.ARR = 65535U};
static TIM_TypeDef tim5_instance = {.ARR = 65535U};

TIM_HandleTypeDef htim2 = {.Instance = &tim2_instance};
TIM_HandleTypeDef htim3 = {.Instance = &tim3_instance};
TIM_HandleTypeDef htim4 = {.Instance = &tim4_instance};
TIM_HandleTypeDef htim5 = {.Instance = &tim5_instance};

static uint32_t fake_primask;
static uint32_t layout_calls_while_masked;

static void WheelEncoderDriverTest_Update(uint32_t now_ms)
{
    param_model_t                 params;
    wheel_encoder_driver_config_t config;

    (void)ParamService_GetSnapshot(&params);
    config.wheel_radius_m = params.wheel_radius_m;
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        config.encoder_dir[index] = params.encoder_dir[index];
    }
    WheelEncoderDriver_Update(now_ms, &config);
}

static void require_int(int condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

static void require_close(float actual, float expected, float tolerance, const char *message)
{
    if (fabsf(actual - expected) > tolerance)
    {
        (void)fprintf(stderr, "FAIL: %s (actual=%f expected=%f)\n", message, actual, expected);
        exit(1);
    }
}

uint32_t __get_PRIMASK(void)
{
    return fake_primask;
}

void __disable_irq(void)
{
    fake_primask = 1U;
}

void __set_PRIMASK(uint32_t primask)
{
    fake_primask = primask;
}

HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t channel)
{
    (void)htim;
    (void)channel;
    return HAL_OK;
}

uint8_t MotorHardwareLayout_MotorEnabled(motor_id_t motor)
{
    if (fake_primask != 0U)
    {
        layout_calls_while_masked++;
    }
    return ((uint32_t)motor < MOTOR_ID_COUNT) ? 1U : 0U;
}

motor_side_t MotorHardwareLayout_MotorSide(motor_id_t motor)
{
    if (fake_primask != 0U)
    {
        layout_calls_while_masked++;
    }
    return ((uint32_t)motor < 2U) ? MOTOR_SIDE_LEFT : MOTOR_SIDE_RIGHT;
}

int8_t MotorHardwareLayout_EncoderDirection(motor_id_t motor)
{
    if (fake_primask != 0U)
    {
        layout_calls_while_masked++;
    }
    (void)motor;
    return 1;
}

static void set_all_counters(uint32_t count)
{
    tim2_instance.CNT = count;
    tim3_instance.CNT = count;
    tim4_instance.CNT = count;
    tim5_instance.CNT = count;
}

static void test_update_publishes_after_unmasked_calculation(void)
{
    wheel_encoder_state_t state;

    ParamService_SetDefaults();
    WheelEncoderDriver_Init();
    set_all_counters(10U);
    WheelEncoderDriverTest_Update(10U);
    layout_calls_while_masked = 0U;
    set_all_counters(20U);
    WheelEncoderDriverTest_Update(20U);
    WheelEncoderDriver_GetState(&state);

    require_int(state.speed_valid_all != 0U, "encoder state becomes valid");
    require_int(layout_calls_while_masked == 0U,
                "layout and filter calculations stay outside the publish critical section");
    require_int(fake_primask == 0U, "encoder update restores interrupt state");
}

static void test_runtime_encoder_direction_reverses_delta(void)
{
    wheel_encoder_state_t state;
    param_model_t         params;

    ParamService_Defaults(&params);
    params.encoder_dir[MOTOR_ID_M2] = -1;
    require_int(ParamService_Set(&params) != 0U, "runtime encoder direction accepted");
    WheelEncoderDriver_Init();
    set_all_counters(100U);
    WheelEncoderDriverTest_Update(10U);
    tim4_instance.CNT = 110U;
    WheelEncoderDriverTest_Update(20U);
    WheelEncoderDriver_GetState(&state);
    require_int(state.delta[MOTOR_ID_M2] == -10, "runtime encoder direction reverses delta");
    ParamService_SetDefaults();
}

static void test_runtime_wheel_radius_changes_speed_generation(void)
{
    wheel_encoder_state_t before;
    wheel_encoder_state_t after;
    param_model_t         params;

    ParamService_SetDefaults();
    WheelEncoderDriver_Init();
    set_all_counters(10U);
    WheelEncoderDriverTest_Update(10U);
    set_all_counters(20U);
    WheelEncoderDriverTest_Update(20U);
    WheelEncoderDriver_GetState(&before);

    (void)ParamService_GetSnapshot(&params);
    params.wheel_radius_m = 0.070f;
    require_int(ParamService_Set(&params) != 0U, "runtime wheel radius accepted");
    set_all_counters(30U);
    WheelEncoderDriverTest_Update(30U);
    WheelEncoderDriver_GetState(&after);

    require_int(before.speed_mps[MOTOR_ID_M1] > 0.0f, "baseline speed is positive");
    require_close(after.speed_mps[MOTOR_ID_M1],
                  before.speed_mps[MOTOR_ID_M1] * 2.0f,
                  0.0001f,
                  "runtime radius doubles converted speed");
}

static void test_hardware_count_snapshot_uses_logical_motor_order(void)
{
    uint32_t counts[MOTOR_ID_COUNT] = {0U};

    tim2_instance.CNT = 12U;
    tim4_instance.CNT = 24U;
    tim3_instance.CNT = 33U;
    tim5_instance.CNT = 45U;
    WheelEncoderDriver_GetHardwareCounts(counts);

    require_int(counts[MOTOR_ID_M1] == 12U, "M1 maps to TIM2");
    require_int(counts[MOTOR_ID_M2] == 24U, "M2 maps to TIM4");
    require_int(counts[MOTOR_ID_M3] == 33U, "M3 maps to TIM3");
    require_int(counts[MOTOR_ID_M4] == 45U, "M4 maps to TIM5");
}

int main(void)
{
    test_runtime_encoder_direction_reverses_delta();
    test_update_publishes_after_unmasked_calculation();
    test_runtime_wheel_radius_changes_speed_generation();
    test_hardware_count_snapshot_uses_logical_motor_order();
    (void)printf("PASS: encoder driver host tests\n");
    return 0;
}
