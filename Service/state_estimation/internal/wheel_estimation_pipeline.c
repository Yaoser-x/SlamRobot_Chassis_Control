#include "wheel_estimation_pipeline.h"

#include "motor_hardware_layout.h"
#include "wheel_encoder_config.h"
#include "wheel_speed_estimator.h"

#define TWO_PI_F 6.28318530718f

static uint32_t               last_count[STATE_ESTIMATION_MOTOR_COUNT];
static encoder_speed_window_t speed_window[STATE_ESTIMATION_MOTOR_COUNT];
static uint32_t               last_update_ms;
static uint8_t                has_last_update;

static float WheelEstimationPipeline_AbsF(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t WheelEstimationPipeline_AbsI32(int32_t value)
{
    return (value < 0) ? -value : value;
}

void WheelEstimationPipeline_Init(void)
{
    for (uint32_t index = 0U; index < STATE_ESTIMATION_MOTOR_COUNT; ++index)
    {
        last_count[index] = 0U;
        WheelSpeedEstimator_SpeedWindowReset(&speed_window[index]);
    }
    last_update_ms  = 0U;
    has_last_update = 0U;
}

void WheelEstimationPipeline_Update(const wheel_encoder_sample_t    *sample,
                                    uint32_t                         now_ms,
                                    float                            wheel_radius_m,
                                    const int8_t                     encoder_dir[STATE_ESTIMATION_MOTOR_COUNT],
                                    state_estimation_wheel_status_t *status)
{
    const float counts_per_rev = CHASSIS_ENCODER_BASE_PPR * CHASSIS_ENCODER_QUADRATURE_MULT
                                 * CHASSIS_MOTOR_GEAR_RATIO;
    const uint32_t dt_ms = now_ms - last_update_ms;
    uint8_t        side_count[2] = {0U, 0U};
    int32_t        side_count_sum[2] = {0, 0};
    int32_t        side_delta_sum[2] = {0, 0};
    float          side_speed_sum[2] = {0.0f, 0.0f};

    if (sample == 0 || encoder_dir == 0 || status == 0)
    {
        return;
    }
    status->last_update_ms        = now_ms;
    status->speed_valid_all       = 1U;
    status->side_consistency_flags = 0UL;

    for (uint32_t index = 0U; index < STATE_ESTIMATION_MOTOR_COUNT; ++index)
    {
        const motor_id_t motor = (motor_id_t)index;
        int32_t          delta = WheelSpeedEstimator_DiffCount(sample->count[index], last_count[index], sample->period[index]);
        const uint8_t    side  = (uint8_t)MotorHardwareLayout_MotorSide(motor);

        last_count[index] = sample->count[index];
        if (encoder_dir[index] < 0)
        {
            delta = -delta;
        }
        if (MotorHardwareLayout_MotorEnabled(motor) == 0U)
        {
            status->count[index]                 = 0;
            status->delta[index]                 = 0;
            status->speed_mps[index]             = 0.0f;
            status->speed_valid[index]           = 0U;
            status->reject_streak[index]         = 0U;
            status->consecutive_anomalies[index] = 0U;
            WheelSpeedEstimator_SpeedWindowReset(&speed_window[index]);
            continue;
        }
        if (has_last_update == 0U || dt_ms <= CHASSIS_MIN_ENCODER_DT_MS || dt_ms > CHASSIS_MAX_ENCODER_DT_MS
            || counts_per_rev <= 0.0f || wheel_radius_m <= 0.0f)
        {
            status->delta[index]         = delta;
            status->count[index] += delta;
            status->speed_mps[index]     = 0.0f;
            status->speed_valid[index]   = 0U;
            status->reject_streak[index] = 0U;
            WheelSpeedEstimator_SpeedWindowReset(&speed_window[index]);
        }
        else if (WheelSpeedEstimator_RecordDeltaOrRebuild(&speed_window[index],
                                                           delta,
                                                           dt_ms,
                                                           counts_per_rev,
                                                           wheel_radius_m,
                                                           CHASSIS_ENCODER_MAX_ABS_MPS,
                                                           CHASSIS_ENCODER_SPIKE_REJECT_MPS,
                                                           CHASSIS_ENCODER_FILTER_MIN_SAMPLES,
                                                           CHASSIS_ENCODER_REBUILD_REJECTS,
                                                           &status->reject_streak[index],
                                                           &status->window_rebuild_count[index]) != 0U)
        {
            status->delta[index] = delta;
            status->count[index] += delta;
            status->consecutive_anomalies[index] = 0U;
            status->speed_mps[index] = WheelSpeedEstimator_CountDeltaSpeedMps(speed_window[index].delta_sum,
                                                                               speed_window[index].dt_sum_ms,
                                                                               counts_per_rev,
                                                                               wheel_radius_m);
            status->speed_valid[index] = 1U;
        }
        else
        {
            status->delta[index] = 0;
            status->anomaly_count[index]++;
            status->consecutive_anomalies[index]++;
            status->speed_valid[index] =
                (status->consecutive_anomalies[index] < CHASSIS_ENCODER_MAX_CONSECUTIVE_ANOMALIES) ? 1U : 0U;
        }
        if (status->speed_valid[index] == 0U)
        {
            status->speed_valid_all = 0U;
        }
        if (side <= (uint8_t)MOTOR_SIDE_RIGHT)
        {
            side_count[side]++;
            side_count_sum[side] += status->count[index];
            side_delta_sum[side] += status->delta[index];
            side_speed_sum[side] += status->speed_mps[index];
        }
    }

    status->left_count       = (side_count[MOTOR_SIDE_LEFT] != 0U) ? side_count_sum[MOTOR_SIDE_LEFT] / side_count[MOTOR_SIDE_LEFT] : 0;
    status->right_count      = (side_count[MOTOR_SIDE_RIGHT] != 0U) ? side_count_sum[MOTOR_SIDE_RIGHT] / side_count[MOTOR_SIDE_RIGHT] : 0;
    status->left_delta       = (side_count[MOTOR_SIDE_LEFT] != 0U) ? side_delta_sum[MOTOR_SIDE_LEFT] / side_count[MOTOR_SIDE_LEFT] : 0;
    status->right_delta      = (side_count[MOTOR_SIDE_RIGHT] != 0U) ? side_delta_sum[MOTOR_SIDE_RIGHT] / side_count[MOTOR_SIDE_RIGHT] : 0;
    status->left_speed_mps   = (side_count[MOTOR_SIDE_LEFT] != 0U) ? side_speed_sum[MOTOR_SIDE_LEFT] / side_count[MOTOR_SIDE_LEFT] : 0.0f;
    status->right_speed_mps  = (side_count[MOTOR_SIDE_RIGHT] != 0U) ? side_speed_sum[MOTOR_SIDE_RIGHT] / side_count[MOTOR_SIDE_RIGHT] : 0.0f;
    status->left_speed_valid = (side_count[MOTOR_SIDE_LEFT] != 0U) ? 1U : 0U;
    status->right_speed_valid = (side_count[MOTOR_SIDE_RIGHT] != 0U) ? 1U : 0U;

    for (uint32_t first = 0U; first < STATE_ESTIMATION_MOTOR_COUNT; ++first)
    {
        for (uint32_t second = first + 1U; second < STATE_ESTIMATION_MOTOR_COUNT; ++second)
        {
            const motor_side_t side = MotorHardwareLayout_MotorSide((motor_id_t)first);
            uint32_t speed_flag;
            uint32_t count_flag;
            uint32_t direction_flag;

            if (MotorHardwareLayout_MotorEnabled((motor_id_t)first) == 0U
                || MotorHardwareLayout_MotorEnabled((motor_id_t)second) == 0U
                || side != MotorHardwareLayout_MotorSide((motor_id_t)second))
            {
                continue;
            }
            speed_flag     = (side == MOTOR_SIDE_LEFT) ? (1UL << 0) : (1UL << 3);
            count_flag     = (side == MOTOR_SIDE_LEFT) ? (1UL << 1) : (1UL << 4);
            direction_flag = (side == MOTOR_SIDE_LEFT) ? (1UL << 2) : (1UL << 5);
            if (WheelEstimationPipeline_AbsF(status->speed_mps[first] - status->speed_mps[second])
                > CHASSIS_ENCODER_SIDE_SPEED_DIFF_MPS)
            {
                status->side_consistency_flags |= speed_flag;
            }
            if (WheelEstimationPipeline_AbsI32(status->count[first] - status->count[second])
                > CHASSIS_ENCODER_SIDE_COUNT_DIFF)
            {
                status->side_consistency_flags |= count_flag;
            }
            if (status->delta[first] != 0 && status->delta[second] != 0
                && ((status->delta[first] > 0 && status->delta[second] < 0)
                    || (status->delta[first] < 0 && status->delta[second] > 0)))
            {
                status->side_consistency_flags |= direction_flag;
            }
        }
    }
    last_update_ms  = now_ms;
    has_last_update = 1U;
}
