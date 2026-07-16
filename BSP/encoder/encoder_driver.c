#include "encoder_driver.h"

#include "encoder_config.h"
#include "chassis_layout.h"
#include "wheel_speed_estimator.h"
#include "direction_apply.h"
#include "tim.h"

#define TWO_PI_F 6.28318530718f

typedef struct
{
    TIM_HandleTypeDef *htim;
} encoder_hw_t;

/* CubeMX labels keep the legacy M2/M3 names; logical IDs are mapped here. */
static const encoder_hw_t encoder_hw[MOTOR_ID_COUNT] = {
    {&htim2},
    {&htim4},
    {&htim3},
    {&htim5},
};

static encoder_state_t        published_encoder_state;
static uint32_t               last_count[MOTOR_ID_COUNT];
static encoder_speed_window_t speed_window[MOTOR_ID_COUNT];
static uint32_t               last_update_ms;
static uint8_t                has_last_update;

static float EncoderDriver_AbsF(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t EncoderDriver_AbsI32(int32_t value)
{
    return (value < 0) ? -value : value;
}

int32_t EncoderDriver_DiffCount(uint32_t now, uint32_t last, uint32_t period)
{
    return WheelSpeedEstimator_DiffCount(now, last, period);
}

float EncoderDriver_GetCountsPerRev(void)
{
    return CHASSIS_ENCODER_BASE_PPR * CHASSIS_ENCODER_QUADRATURE_MULT * CHASSIS_MOTOR_GEAR_RATIO;
}

void EncoderDriver_GetHardwareCounts(uint32_t counts[MOTOR_ID_COUNT])
{
    if (counts == 0)
    {
        return;
    }
    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        counts[i] = __HAL_TIM_GET_COUNTER(encoder_hw[i].htim);
    }
}

void EncoderDriver_Init(void)
{
    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        (void)HAL_TIM_Encoder_Start(encoder_hw[i].htim, TIM_CHANNEL_ALL);
        __HAL_TIM_SET_COUNTER(encoder_hw[i].htim, 0U);
        last_count[i] = 0U;
        WheelSpeedEstimator_SpeedWindowReset(&speed_window[i]);
    }
    published_encoder_state = (encoder_state_t){0};
    last_update_ms          = 0U;
    has_last_update         = 0U;
}

void EncoderDriver_Update(uint32_t now_ms, const encoder_driver_config_t *config)
{
    encoder_state_t encoder_state;
    uint32_t        now_count[MOTOR_ID_COUNT];
    int32_t         delta[MOTOR_ID_COUNT];
    uint32_t        dt_ms          = now_ms - last_update_ms;
    float           counts_per_rev = EncoderDriver_GetCountsPerRev();
    float           meters_per_rev;
    uint32_t        primask;
    uint8_t         valid_all       = 1U;
    uint8_t         left_count      = 0U;
    uint8_t         right_count     = 0U;
    int32_t         left_count_sum  = 0;
    int32_t         right_count_sum = 0;
    int32_t         left_delta_sum  = 0;
    int32_t         right_delta_sum = 0;
    float           left_speed_sum  = 0.0f;
    float           right_speed_sum = 0.0f;

    if (config == 0)
    {
        return;
    }
    meters_per_rev = TWO_PI_F * config->wheel_radius_m;

    primask = __get_PRIMASK();
    __disable_irq();
    encoder_state = published_encoder_state;
    __set_PRIMASK(primask);

    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        uint32_t period = __HAL_TIM_GET_AUTORELOAD(encoder_hw[i].htim);
        now_count[i]    = __HAL_TIM_GET_COUNTER(encoder_hw[i].htim);
        if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U)
        {
            delta[i] = DirectionApply_Signed(EncoderDriver_DiffCount(now_count[i], last_count[i], period),
                                             config->encoder_dir[i]);
        }
        else
        {
            delta[i] = 0;
        }
        last_count[i] = now_count[i];
    }

    encoder_state.last_update_ms = now_ms;
    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (ChassisLayout_MotorEnabled((motor_id_t)i) == 0U)
        {
            encoder_state.count[i]         = 0;
            encoder_state.delta[i]         = 0;
            encoder_state.speed_mps[i]     = 0.0f;
            encoder_state.speed_valid[i]   = 0U;
            encoder_state.reject_streak[i] = 0U;
            WheelSpeedEstimator_SpeedWindowReset(&speed_window[i]);
            continue;
        }

        if (has_last_update == 0U || dt_ms <= CHASSIS_MIN_ENCODER_DT_MS || dt_ms > CHASSIS_MAX_ENCODER_DT_MS
            || counts_per_rev <= 0.0f || meters_per_rev <= 0.0f)
        {
            encoder_state.delta[i] = delta[i];
            encoder_state.count[i] += delta[i];
            encoder_state.speed_mps[i]     = 0.0f;
            encoder_state.speed_valid[i]   = 0U;
            encoder_state.reject_streak[i] = 0U;
            WheelSpeedEstimator_SpeedWindowReset(&speed_window[i]);
            valid_all = 0U;
        }
        else
        {
            if (WheelSpeedEstimator_RecordDeltaOrRebuild(&speed_window[i],
                                                         delta[i],
                                                         dt_ms,
                                                         counts_per_rev,
                                                         config->wheel_radius_m,
                                                         CHASSIS_ENCODER_MAX_ABS_MPS,
                                                         CHASSIS_ENCODER_SPIKE_REJECT_MPS,
                                                         CHASSIS_ENCODER_FILTER_MIN_SAMPLES,
                                                         CHASSIS_ENCODER_REBUILD_REJECTS,
                                                         &encoder_state.reject_streak[i],
                                                         &encoder_state.window_rebuild_count[i])
                != 0U)
            {
                encoder_state.delta[i] = delta[i];
                encoder_state.count[i] += delta[i];
                encoder_state.consecutive_anomalies[i] = 0U;
            }
            else
            {
                encoder_state.delta[i] = 0;
                encoder_state.anomaly_count[i]++;
                encoder_state.consecutive_anomalies[i]++;
            }

            encoder_state.speed_mps[i] = WheelSpeedEstimator_CountDeltaSpeedMps(speed_window[i].delta_sum,
                                                                                speed_window[i].dt_sum_ms,
                                                                                counts_per_rev,
                                                                                config->wheel_radius_m);
            if (encoder_state.consecutive_anomalies[i] >= CHASSIS_ENCODER_MAX_CONSECUTIVE_ANOMALIES)
            {
                encoder_state.speed_valid[i] = 0U;
            }
            else
            {
                encoder_state.speed_valid[i] = (speed_window[i].dt_sum_ms != 0U) ? 1U : 0U;
            }
        }

        if (ChassisLayout_MotorSide((motor_id_t)i) == MOTOR_SIDE_LEFT)
        {
            left_count++;
            left_count_sum += encoder_state.count[i];
            left_delta_sum += encoder_state.delta[i];
            left_speed_sum += encoder_state.speed_mps[i];
        }
        else
        {
            right_count++;
            right_count_sum += encoder_state.count[i];
            right_delta_sum += encoder_state.delta[i];
            right_speed_sum += encoder_state.speed_mps[i];
        }
    }

    encoder_state.left_count             = (left_count != 0U) ? (left_count_sum / (int32_t)left_count) : 0;
    encoder_state.right_count            = (right_count != 0U) ? (right_count_sum / (int32_t)right_count) : 0;
    encoder_state.left_delta             = (left_count != 0U) ? (left_delta_sum / (int32_t)left_count) : 0;
    encoder_state.right_delta            = (right_count != 0U) ? (right_delta_sum / (int32_t)right_count) : 0;
    encoder_state.left_speed_mps         = (left_count != 0U) ? (left_speed_sum / (float)left_count) : 0.0f;
    encoder_state.right_speed_mps        = (right_count != 0U) ? (right_speed_sum / (float)right_count) : 0.0f;
    encoder_state.left_speed_valid       = (left_count != 0U) ? 1U : 0U;
    encoder_state.right_speed_valid      = (right_count != 0U) ? 1U : 0U;
    encoder_state.side_consistency_flags = 0UL;
    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (ChassisLayout_MotorEnabled((motor_id_t)i) == 0U)
        {
            continue;
        }
        if (ChassisLayout_MotorSide((motor_id_t)i) == MOTOR_SIDE_LEFT && encoder_state.speed_valid[i] == 0U)
        {
            encoder_state.left_speed_valid = 0U;
        }
        if (ChassisLayout_MotorSide((motor_id_t)i) == MOTOR_SIDE_RIGHT && encoder_state.speed_valid[i] == 0U)
        {
            encoder_state.right_speed_valid = 0U;
        }
    }
    if (left_count >= 2U)
    {
        float   left_a_speed = 0.0f;
        float   left_b_speed = 0.0f;
        int32_t left_a_count = 0;
        int32_t left_b_count = 0;
        int32_t left_a_delta = 0;
        int32_t left_b_delta = 0;
        uint8_t found        = 0U;

        for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
        {
            if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U
                && ChassisLayout_MotorSide((motor_id_t)i) == MOTOR_SIDE_LEFT)
            {
                if (found == 0U)
                {
                    left_a_speed = encoder_state.speed_mps[i];
                    left_a_count = encoder_state.count[i];
                    left_a_delta = encoder_state.delta[i];
                }
                else
                {
                    left_b_speed = encoder_state.speed_mps[i];
                    left_b_count = encoder_state.count[i];
                    left_b_delta = encoder_state.delta[i];
                }
                found++;
            }
        }
        if (found >= 2U)
        {
            if (EncoderDriver_AbsF(left_a_speed - left_b_speed) > CHASSIS_ENCODER_SIDE_SPEED_DIFF_MPS)
            {
                encoder_state.side_consistency_flags |= ENCODER_SIDE_CONSISTENCY_LEFT_SPEED;
            }
            if (EncoderDriver_AbsI32(left_a_count - left_b_count) > CHASSIS_ENCODER_SIDE_COUNT_DIFF)
            {
                encoder_state.side_consistency_flags |= ENCODER_SIDE_CONSISTENCY_LEFT_COUNT;
            }
            if (left_a_delta != 0 && left_b_delta != 0
                && ((left_a_delta > 0 && left_b_delta < 0) || (left_a_delta < 0 && left_b_delta > 0)))
            {
                encoder_state.side_consistency_flags |= ENCODER_SIDE_CONSISTENCY_LEFT_DIRECTION;
            }
        }
    }
    if (right_count >= 2U)
    {
        float   right_a_speed = 0.0f;
        float   right_b_speed = 0.0f;
        int32_t right_a_count = 0;
        int32_t right_b_count = 0;
        int32_t right_a_delta = 0;
        int32_t right_b_delta = 0;
        uint8_t found         = 0U;

        for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
        {
            if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U
                && ChassisLayout_MotorSide((motor_id_t)i) == MOTOR_SIDE_RIGHT)
            {
                if (found == 0U)
                {
                    right_a_speed = encoder_state.speed_mps[i];
                    right_a_count = encoder_state.count[i];
                    right_a_delta = encoder_state.delta[i];
                }
                else
                {
                    right_b_speed = encoder_state.speed_mps[i];
                    right_b_count = encoder_state.count[i];
                    right_b_delta = encoder_state.delta[i];
                }
                found++;
            }
        }
        if (found >= 2U)
        {
            if (EncoderDriver_AbsF(right_a_speed - right_b_speed) > CHASSIS_ENCODER_SIDE_SPEED_DIFF_MPS)
            {
                encoder_state.side_consistency_flags |= ENCODER_SIDE_CONSISTENCY_RIGHT_SPEED;
            }
            if (EncoderDriver_AbsI32(right_a_count - right_b_count) > CHASSIS_ENCODER_SIDE_COUNT_DIFF)
            {
                encoder_state.side_consistency_flags |= ENCODER_SIDE_CONSISTENCY_RIGHT_COUNT;
            }
            if (right_a_delta != 0 && right_b_delta != 0
                && ((right_a_delta > 0 && right_b_delta < 0) || (right_a_delta < 0 && right_b_delta > 0)))
            {
                encoder_state.side_consistency_flags |= ENCODER_SIDE_CONSISTENCY_RIGHT_DIRECTION;
            }
        }
    }
    if (encoder_state.left_speed_valid == 0U || encoder_state.right_speed_valid == 0U)
    {
        valid_all = 0U;
    }
    encoder_state.speed_valid_all = valid_all;

    primask = __get_PRIMASK();
    __disable_irq();
    published_encoder_state = encoder_state;
    __set_PRIMASK(primask);

    last_update_ms  = now_ms;
    has_last_update = 1U;
}

void EncoderDriver_GetState(encoder_state_t *state)
{
    uint32_t primask;

    if (state == 0)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *state = published_encoder_state;
    __set_PRIMASK(primask);
}

float EncoderDriver_GetMotorSpeedMps(motor_id_t motor)
{
    uint32_t primask;
    float    speed;

    if ((uint32_t)motor >= MOTOR_ID_COUNT)
    {
        return 0.0f;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    speed = published_encoder_state.speed_mps[(uint32_t)motor];
    __set_PRIMASK(primask);
    return speed;
}
