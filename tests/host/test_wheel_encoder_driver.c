#include "wheel_encoder_driver.h"
#include "wheel_estimation_pipeline.h"

#include "motor_hardware_layout.h"
#include "param_service.h"
#include "tim.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static TIM_TypeDef tim2_instance = {.ARR = 65535U};
static TIM_TypeDef tim3_instance = {.ARR = 65535U};
static TIM_TypeDef tim4_instance = {.ARR = 65535U};
static TIM_TypeDef tim5_instance = {.ARR = 65535U};

TIM_HandleTypeDef htim2 = {.Instance = &tim2_instance};
TIM_HandleTypeDef htim3 = {.Instance = &tim3_instance};
TIM_HandleTypeDef htim4 = {.Instance = &tim4_instance};
TIM_HandleTypeDef htim5 = {.Instance = &tim5_instance};

static uint32_t                        fake_primask;
static uint32_t                        layout_calls_while_masked;
static uint8_t                         enabled_mask = 0x0FU;
static state_estimation_wheel_status_t wheel_status;

static void WheelEncoderDriverTest_Init(void)
{
    WheelEncoderDriver_Init();
    WheelEstimationPipeline_Init();
    wheel_status = (state_estimation_wheel_status_t){0};
}

static void WheelEncoderDriverTest_Update(uint32_t now_ms)
{
    param_model_t          params;
    wheel_encoder_sample_t sample;

    (void)ParamService_GetSnapshot(&params);
    WheelEncoderDriver_Read(&sample);
    WheelEstimationPipeline_Update(&sample, now_ms, params.wheel_radius_m, params.encoder_dir, &wheel_status);
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
    return ((uint32_t)motor < MOTOR_ID_COUNT && (enabled_mask & (uint8_t)(1U << (uint8_t)motor)) != 0U) ? 1U : 0U;
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
    state_estimation_wheel_status_t state;

    ParamService_SetDefaults();
    WheelEncoderDriverTest_Init();
    set_all_counters(10U);
    WheelEncoderDriverTest_Update(10U);
    layout_calls_while_masked = 0U;
    set_all_counters(20U);
    WheelEncoderDriverTest_Update(20U);
    state = wheel_status;

    require_int(state.speed_valid_all != 0U, "encoder state becomes valid");
    require_int(layout_calls_while_masked == 0U,
                "layout and filter calculations stay outside the publish critical section");
    require_int(fake_primask == 0U, "encoder update restores interrupt state");
}

static void test_runtime_encoder_direction_reverses_delta(void)
{
    state_estimation_wheel_status_t state;
    param_model_t                   params;

    ParamService_Defaults(&params);
    params.encoder_dir[MOTOR_ID_M2] = -1;
    require_int(ParamService_Set(&params) != 0U, "runtime encoder direction accepted");
    WheelEncoderDriverTest_Init();
    set_all_counters(100U);
    WheelEncoderDriverTest_Update(10U);
    tim4_instance.CNT = 110U;
    WheelEncoderDriverTest_Update(20U);
    state = wheel_status;
    require_int(state.delta[MOTOR_ID_M2] == -10, "runtime encoder direction reverses delta");
    ParamService_SetDefaults();
}

static void test_runtime_wheel_radius_changes_speed_generation(void)
{
    state_estimation_wheel_status_t before;
    state_estimation_wheel_status_t after;
    param_model_t                   params;

    ParamService_SetDefaults();
    WheelEncoderDriverTest_Init();
    set_all_counters(10U);
    WheelEncoderDriverTest_Update(10U);
    set_all_counters(20U);
    WheelEncoderDriverTest_Update(20U);
    before = wheel_status;

    (void)ParamService_GetSnapshot(&params);
    params.wheel_radius_m = 0.070f;
    require_int(ParamService_Set(&params) != 0U, "runtime wheel radius accepted");
    set_all_counters(30U);
    WheelEncoderDriverTest_Update(30U);
    after = wheel_status;

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

static void test_side_validity_requires_every_enabled_encoder(void)
{
    uint8_t speed_valid[MOTOR_ID_COUNT] = {1U, 1U, 1U, 1U};
    uint8_t left_valid;
    uint8_t right_valid;

    enabled_mask             = 0x0FU;
    speed_valid[MOTOR_ID_M2] = 0U;
    WheelEstimationPipeline_AggregateSideValidity(speed_valid, &left_valid, &right_valid);
    require_int(left_valid == 0U && right_valid != 0U, "one invalid left encoder invalidates only the left side");
    speed_valid[MOTOR_ID_M2] = 1U;
    speed_valid[MOTOR_ID_M3] = 0U;
    WheelEstimationPipeline_AggregateSideValidity(speed_valid, &left_valid, &right_valid);
    require_int(left_valid != 0U && right_valid == 0U, "one invalid right encoder invalidates only the right side");
    speed_valid[MOTOR_ID_M2] = 0U;
    enabled_mask &= (uint8_t) ~(1U << MOTOR_ID_M2);
    WheelEstimationPipeline_AggregateSideValidity(speed_valid, &left_valid, &right_valid);
    require_int(left_valid != 0U, "disabled invalid motor does not invalidate its side");
    enabled_mask = 0x0FU;
}

static uint32_t count_bits(int32_t count)
{
    uint32_t bits;

    memcpy(&bits, &count, sizeof(bits));
    return bits;
}

static void test_raw_cumulative_count_and_anomaly_delivery_latch(void)
{
    uint32_t first_generation;

    ParamService_SetDefaults();
    WheelEncoderDriverTest_Init();
    set_all_counters(0U);
    WheelEncoderDriverTest_Update(10U);
    set_all_counters(10U);
    WheelEncoderDriverTest_Update(20U);

    tim2_instance.CNT = 30000U;
    tim4_instance.CNT = 20U;
    tim3_instance.CNT = 20U;
    tim5_instance.CNT = 20U;
    WheelEncoderDriverTest_Update(30U);
    require_int(wheel_status.delta[MOTOR_ID_M1] == 0, "rejected spike does not enter wheel-speed delta");
    require_int(count_bits(wheel_status.count[MOTOR_ID_M1]) == 30000U,
                "rejected spike remains in the raw cumulative count");
    require_int((wheel_status.current_anomaly_mask & (1U << MOTOR_ID_M1)) != 0U,
                "current anomaly marks the rejected wheel");
    require_int((wheel_status.latched_for_host_mask & (1U << MOTOR_ID_M1)) != 0U,
                "anomaly remains latched for Host STATUS");
    first_generation = wheel_status.anomaly_delivery_generation;
    require_int(first_generation != 0UL, "new anomaly advances delivery generation");

    tim2_instance.CNT = 30010U;
    tim4_instance.CNT = 30U;
    tim3_instance.CNT = 30U;
    tim5_instance.CNT = 30U;
    WheelEncoderDriverTest_Update(40U);
    require_int((wheel_status.current_anomaly_mask & (1U << MOTOR_ID_M1)) == 0U,
                "stable sample clears the current anomaly");
    require_int((wheel_status.latched_for_host_mask & (1U << MOTOR_ID_M1)) != 0U,
                "cleared current anomaly stays latched until delivery");
    require_int(WheelEstimationPipeline_AcknowledgeAnomalyDelivery(&wheel_status, first_generation - 1UL) == 0U,
                "stale delivery generation cannot clear the latch");
    require_int(WheelEstimationPipeline_AcknowledgeAnomalyDelivery(&wheel_status, first_generation) != 0U,
                "matching delivery generation clears the latch");
    require_int(wheel_status.latched_for_host_mask == 0U, "matching delivery clears all included anomaly bits");

    tim2_instance.CNT = 60000U;
    WheelEncoderDriverTest_Update(50U);
    require_int(wheel_status.anomaly_delivery_generation != first_generation,
                "a later anomaly advances generation again");
    require_int(WheelEstimationPipeline_AcknowledgeAnomalyDelivery(&wheel_status, first_generation) == 0U,
                "old TX completion cannot clear a newer anomaly");
}

static void test_raw_cumulative_count_wraps_modulo_32_bits(void)
{
    ParamService_SetDefaults();
    WheelEncoderDriverTest_Init();
    set_all_counters(65530U);
    WheelEncoderDriverTest_Update(10U);
    set_all_counters(5U);
    WheelEncoderDriverTest_Update(20U);
    require_int(count_bits(wheel_status.count[MOTOR_ID_M1]) == 5U,
                "raw cumulative count follows modular timer movement across wrap");
}

int main(void)
{
    test_runtime_encoder_direction_reverses_delta();
    test_update_publishes_after_unmasked_calculation();
    test_runtime_wheel_radius_changes_speed_generation();
    test_hardware_count_snapshot_uses_logical_motor_order();
    test_side_validity_requires_every_enabled_encoder();
    test_raw_cumulative_count_and_anomaly_delivery_latch();
    test_raw_cumulative_count_wraps_modulo_32_bits();
    (void)printf("PASS: encoder driver host tests\n");
    return 0;
}
