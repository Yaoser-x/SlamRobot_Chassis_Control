#include "adc_monitor.h"
#include "bsp_config.h"

#include "adc.h"
#include "bsp_config.h"
#include "control_config.h"
#include "chassis_layout.h"
#include "tim.h"

#include <string.h>

static uint16_t            adc_dma_buffer[ADC_MONITOR_CHANNEL_COUNT];
static volatile uint16_t   adc_sample_snapshot[ADC_MONITOR_CHANNEL_COUNT];
static volatile uint16_t   adc_window_latest_raw[MOTOR_ID_COUNT];
static volatile uint32_t   adc_window_raw_sum[MOTOR_ID_COUNT];
static volatile uint16_t   adc_window_raw_min[MOTOR_ID_COUNT];
static volatile uint16_t   adc_window_raw_max[MOTOR_ID_COUNT];
static volatile uint32_t   adc_window_delta_sum[MOTOR_ID_COUNT];
static volatile uint64_t   adc_window_delta_sq_sum[MOTOR_ID_COUNT];
static volatile int64_t    adc_window_signed_delta_sum[MOTOR_ID_COUNT];
static volatile uint64_t   adc_window_signed_delta_sq_sum[MOTOR_ID_COUNT];
static volatile uint16_t   adc_window_delta_min[MOTOR_ID_COUNT];
static volatile uint16_t   adc_window_delta_max[MOTOR_ID_COUNT];
static volatile uint32_t   adc_window_raw_count;
static volatile uint32_t   adc_window_delta_count;
static volatile uint8_t    adc_samples_ready;
static volatile uint8_t    adc_dma_error_latched;
static adc_monitor_state_t adc_state;
static uint8_t             current_filter_initialized;
static uint8_t             battery_filter_initialized;
static uint8_t             current_zero_valid;
static volatile uint8_t    current_zero_stationary;
static uint16_t            current_zero_sample_count;
static uint16_t            current_zero_raw[MOTOR_ID_COUNT];
static uint16_t            current_zero_min_raw[MOTOR_ID_COUNT];
static uint16_t            current_zero_max_raw[MOTOR_ID_COUNT];
static uint16_t            current_zero_span_raw[MOTOR_ID_COUNT];
static uint32_t            current_zero_sum[MOTOR_ID_COUNT];

/* ADC DMA order follows logical motor current channels: M1, M2, M3, M4. */
static const uint8_t current_adc_index[MOTOR_ID_COUNT] = {
    0U,
    1U,
    2U,
    3U,
};

static float AdcMonitor_RawToVoltage(uint16_t raw)
{
    return ((float)raw * ADC_MONITOR_VREF_V) / ADC_MONITOR_RESOLUTION_COUNTS;
}

static float AdcMonitor_RawDeltaToCurrentForMotor(motor_id_t motor, float raw_delta)
{
    static const float volts_per_amp[MOTOR_ID_COUNT] = {
        MOTOR_CURRENT_VOLTS_PER_AMP_M1,
        MOTOR_CURRENT_VOLTS_PER_AMP_M2,
        MOTOR_CURRENT_VOLTS_PER_AMP_M3,
        MOTOR_CURRENT_VOLTS_PER_AMP_M4,
    };
    float scale = MOTOR_CURRENT_VOLTS_PER_AMP;

    if ((uint32_t)motor < MOTOR_ID_COUNT)
    {
        scale = volts_per_amp[(uint32_t)motor];
    }
    if (scale <= 0.0f)
    {
        return 0.0f;
    }
    return (raw_delta * ADC_MONITOR_VREF_V) / (ADC_MONITOR_RESOLUTION_COUNTS * scale);
}

static float AdcMonitor_Sqrtf(float value)
{
    float estimate;

    if (value <= 0.0f)
    {
        return 0.0f;
    }

    estimate = (value >= 1.0f) ? value : 1.0f;
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        estimate = 0.5f * (estimate + (value / estimate));
    }
    return estimate;
}

static uint16_t AdcMonitor_ClampU16(uint32_t value)
{
    return (value > 65535UL) ? 65535U : (uint16_t)value;
}

static float AdcMonitor_FilterBattery(float previous, float sample)
{
    float alpha = ADC_MONITOR_BATTERY_FILTER_ALPHA;

    if (alpha <= 0.0f)
    {
        return previous;
    }
    if (alpha >= 1.0f || battery_filter_initialized == 0U)
    {
        return sample;
    }
    return previous + (alpha * (sample - previous));
}

static void AdcMonitor_ResetWindowAccumulators(void)
{
    adc_window_raw_count   = 0UL;
    adc_window_delta_count = 0UL;
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        adc_window_raw_sum[i]             = 0UL;
        adc_window_raw_min[i]             = 0U;
        adc_window_raw_max[i]             = 0U;
        adc_window_delta_sum[i]           = 0UL;
        adc_window_delta_sq_sum[i]        = 0ULL;
        adc_window_signed_delta_sum[i]    = 0;
        adc_window_signed_delta_sq_sum[i] = 0ULL;
        adc_window_delta_min[i]           = 0U;
        adc_window_delta_max[i]           = 0U;
    }
}

static void AdcMonitor_ResetCurrentZeroAccumulators(void)
{
    current_zero_sample_count = 0U;
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        current_zero_raw[i]      = 0U;
        current_zero_min_raw[i]  = 0U;
        current_zero_max_raw[i]  = 0U;
        current_zero_span_raw[i] = 0U;
        current_zero_sum[i]      = 0UL;
    }
}

static void AdcMonitor_UpdateCurrentZero(const uint32_t raw_sum[MOTOR_ID_COUNT],
                                         const uint16_t raw_min[MOTOR_ID_COUNT],
                                         const uint16_t raw_max[MOTOR_ID_COUNT],
                                         uint32_t       sample_count)
{
    if (ADC_MONITOR_CALIBRATION_ENABLED == 0U || current_zero_valid != 0U || current_zero_stationary == 0U)
    {
        return;
    }
    if (sample_count == 0UL)
    {
        return;
    }

    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        current_zero_sum[i] += raw_sum[i];
        if (current_zero_sample_count == 0U || raw_min[i] < current_zero_min_raw[i])
        {
            current_zero_min_raw[i] = raw_min[i];
        }
        if (current_zero_sample_count == 0U || raw_max[i] > current_zero_max_raw[i])
        {
            current_zero_max_raw[i] = raw_max[i];
        }
    }

    {
        uint32_t total_count = (uint32_t)current_zero_sample_count + sample_count;
        if (total_count >= ADC_MONITOR_CURRENT_ZERO_SAMPLES)
        {
            for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
            {
                current_zero_raw[i]      = (uint16_t)(current_zero_sum[i] / total_count);
                current_zero_span_raw[i] = (current_zero_max_raw[i] >= current_zero_min_raw[i])
                                               ? (uint16_t)(current_zero_max_raw[i] - current_zero_min_raw[i])
                                               : 0U;
            }
            current_zero_sample_count = ADC_MONITOR_CURRENT_ZERO_SAMPLES;
            current_zero_valid        = 1U;
        }
        else
        {
            current_zero_sample_count = (uint16_t)total_count;
        }
    }
}

static void AdcMonitor_ComputeCurrentWindow(uint32_t   delta_sum,
                                            uint64_t   delta_sq_sum,
                                            int64_t    signed_delta_sum,
                                            uint64_t   signed_delta_sq_sum,
                                            uint16_t   delta_min,
                                            uint16_t   delta_max,
                                            uint32_t   sample_count,
                                            motor_id_t motor,
                                            float     *mean_a,
                                            float     *rms_a,
                                            float     *peak_a,
                                            float     *signed_mean_a,
                                            float     *noise_a,
                                            float     *trimmed_a)
{
    float    mean_delta;
    float    rms_delta;
    float    signed_mean_delta;
    float    noise_delta;
    float    trimmed_delta;
    uint32_t trimmed_sum;

    if (sample_count == 0UL)
    {
        *mean_a        = 0.0f;
        *rms_a         = 0.0f;
        *peak_a        = 0.0f;
        *signed_mean_a = 0.0f;
        *noise_a       = 0.0f;
        *trimmed_a     = 0.0f;
        return;
    }

    mean_delta        = (float)delta_sum / (float)sample_count;
    rms_delta         = AdcMonitor_Sqrtf((float)delta_sq_sum / (float)sample_count);
    signed_mean_delta = (float)signed_delta_sum / (float)sample_count;
    {
        float signed_mean_sq = signed_mean_delta * signed_mean_delta;
        float signed_sq_mean = (float)signed_delta_sq_sum / (float)sample_count;
        noise_delta          = AdcMonitor_Sqrtf(signed_sq_mean - signed_mean_sq);
    }
    if (sample_count > 2UL)
    {
        trimmed_sum   = delta_sum - (uint32_t)delta_min - (uint32_t)delta_max;
        trimmed_delta = (float)trimmed_sum / (float)(sample_count - 2UL);
    }
    else
    {
        trimmed_delta = mean_delta;
    }

    *mean_a        = AdcMonitor_RawDeltaToCurrentForMotor(motor, mean_delta);
    *rms_a         = AdcMonitor_RawDeltaToCurrentForMotor(motor, rms_delta);
    *peak_a        = AdcMonitor_RawDeltaToCurrentForMotor(motor, (float)delta_max);
    *signed_mean_a = AdcMonitor_RawDeltaToCurrentForMotor(motor, signed_mean_delta);
    *noise_a       = AdcMonitor_RawDeltaToCurrentForMotor(motor, noise_delta);
    *trimmed_a     = AdcMonitor_RawDeltaToCurrentForMotor(motor, trimmed_delta);
}

static float AdcMonitor_FilterCurrent(float previous, float sample)
{
    float alpha = MOTOR_CURRENT_FILTER_ALPHA;

    if (alpha <= 0.0f)
    {
        return previous;
    }
    if (alpha >= 1.0f || current_filter_initialized == 0U)
    {
        return sample;
    }
    return previous + (alpha * (sample - previous));
}

void AdcMonitor_Init(void)
{
    adc_state = (adc_monitor_state_t){0};
    for (uint8_t i = 0U; i < ADC_MONITOR_CHANNEL_COUNT; ++i)
    {
        adc_sample_snapshot[i] = 0U;
    }
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        adc_window_latest_raw[i] = 0U;
        current_zero_raw[i]      = 0U;
        current_zero_min_raw[i]  = 0U;
        current_zero_max_raw[i]  = 0U;
        current_zero_span_raw[i] = 0U;
        current_zero_sum[i]      = 0U;
    }
    AdcMonitor_ResetWindowAccumulators();
    adc_samples_ready          = 0U;
    adc_dma_error_latched      = 0U;
    current_filter_initialized = 0U;
    battery_filter_initialized = 0U;
    current_zero_valid         = 0U;
    current_zero_stationary    = 0U;
    current_zero_sample_count  = 0U;
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_MONITOR_CHANNEL_COUNT) == HAL_OK)
    {
        /* Keep TC enabled: HAL_ADC_ConvCpltCallback publishes ADC samples. */
        __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
        (void)HAL_TIM_Base_Start(&htim8);
    }
}

void AdcMonitor_Update(void)
{
    /* Called only by safetyTask. Keep its sizeable snapshot workspace out of the
     task stack so ADC diagnostics cannot exhaust the safety margin. */
    static uint16_t            raw_current[MOTOR_ID_COUNT];
    static uint32_t            raw_sum[MOTOR_ID_COUNT];
    static uint16_t            raw_min[MOTOR_ID_COUNT];
    static uint16_t            raw_max[MOTOR_ID_COUNT];
    static uint32_t            delta_sum[MOTOR_ID_COUNT];
    static uint64_t            delta_sq_sum[MOTOR_ID_COUNT];
    static int64_t             signed_delta_sum[MOTOR_ID_COUNT];
    static uint64_t            signed_delta_sq_sum[MOTOR_ID_COUNT];
    static uint16_t            delta_min[MOTOR_ID_COUNT];
    static uint16_t            delta_max[MOTOR_ID_COUNT];
    static uint16_t            zero_raw[MOTOR_ID_COUNT];
    static uint16_t            zero_span_raw[MOTOR_ID_COUNT];
    static uint16_t            raw_battery;
    static adc_monitor_state_t next_state;
    static float               previous_current[MOTOR_ID_COUNT];
    static float               previous_battery;
    static uint8_t             samples_ready;
    static uint32_t            raw_sample_count;
    static uint32_t            delta_sample_count;
    static uint8_t             zero_valid;
    static uint16_t            zero_sample_count;
    static uint8_t             next_current_valid;
    static uint8_t             next_current_control_valid;
    static uint8_t             dma_error_latched;
    static uint32_t            primask;
    static uint8_t             left_count;
    static uint8_t             right_count;
    static uint32_t            left_raw_sum;
    static uint32_t            right_raw_sum;
    static float               left_current_sum;
    static float               right_current_sum;

    left_count        = 0U;
    right_count       = 0U;
    left_raw_sum      = 0U;
    right_raw_sum     = 0U;
    left_current_sum  = 0.0f;
    right_current_sum = 0.0f;

    primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        raw_current[i]         = adc_window_latest_raw[i];
        raw_sum[i]             = adc_window_raw_sum[i];
        raw_min[i]             = adc_window_raw_min[i];
        raw_max[i]             = adc_window_raw_max[i];
        delta_sum[i]           = adc_window_delta_sum[i];
        delta_sq_sum[i]        = adc_window_delta_sq_sum[i];
        signed_delta_sum[i]    = adc_window_signed_delta_sum[i];
        signed_delta_sq_sum[i] = adc_window_signed_delta_sq_sum[i];
        delta_min[i]           = adc_window_delta_min[i];
        delta_max[i]           = adc_window_delta_max[i];
        zero_raw[i]            = current_zero_raw[i];
        zero_span_raw[i]       = current_zero_span_raw[i];
        previous_current[i]    = adc_state.current_a[i];
    }
    raw_battery           = adc_sample_snapshot[4];
    previous_battery      = adc_state.battery_voltage;
    samples_ready         = adc_samples_ready;
    raw_sample_count      = adc_window_raw_count;
    delta_sample_count    = adc_window_delta_count;
    dma_error_latched     = adc_dma_error_latched;
    adc_dma_error_latched = 0U;
    AdcMonitor_ResetWindowAccumulators();
    zero_valid        = current_zero_valid;
    zero_sample_count = current_zero_sample_count;
    __set_PRIMASK(primask);

    if (samples_ready != 0U)
    {
        AdcMonitor_UpdateCurrentZero(raw_sum, raw_min, raw_max, raw_sample_count);
    }

    primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        zero_raw[i]      = current_zero_raw[i];
        zero_span_raw[i] = current_zero_span_raw[i];
    }
    zero_valid        = current_zero_valid;
    zero_sample_count = current_zero_sample_count;
    __set_PRIMASK(primask);

    memset(&next_state, 0, sizeof(next_state));
    next_state.raw_battery = raw_battery;
    next_state.battery_voltage =
        AdcMonitor_FilterBattery(previous_battery, AdcMonitor_RawToVoltage(raw_battery) * ADC_MONITOR_BATTERY_DIVIDER);
    next_state.samples_ready             = samples_ready;
    next_state.current_zero_valid        = zero_valid;
    next_state.current_zero_sample_count = zero_sample_count;
    next_state.raw_sample_count          = AdcMonitor_ClampU16(raw_sample_count);
    next_state.missed_window_count       = adc_state.missed_window_count;
    if (CHASSIS_ADC_PERIOD_MS > 0U)
    {
        next_state.sample_rate_hz_milli = (uint32_t)((raw_sample_count * 1000000UL) / CHASSIS_ADC_PERIOD_MS);
    }
    if (samples_ready != 0U)
    {
        next_state.valid_flags |= ADC_MONITOR_VALID_SAMPLES_READY;
    }
    if (zero_valid != 0U)
    {
        next_state.valid_flags |= ADC_MONITOR_VALID_CURRENT_ZERO_READY;
    }
    if (samples_ready == 0U)
    {
        next_state.invalid_reason_flags |= ADC_MONITOR_INVALID_NOT_READY;
    }
    if (zero_valid == 0U)
    {
        next_state.invalid_reason_flags |= ADC_MONITOR_INVALID_ZERO_CALIBRATING;
    }
    if (raw_sample_count == 0UL)
    {
        next_state.invalid_reason_flags |= ADC_MONITOR_INVALID_NO_NEW_SAMPLE;
        if (zero_valid != 0U)
        {
            next_state.missed_window_count = AdcMonitor_ClampU16((uint32_t)next_state.missed_window_count + 1UL);
        }
    }
    if (dma_error_latched != 0U)
    {
        next_state.invalid_reason_flags |= ADC_MONITOR_INVALID_DMA_ERROR;
    }
    if (zero_valid != 0U && delta_sample_count == 0UL)
    {
        next_state.invalid_reason_flags |= ADC_MONITOR_INVALID_SAMPLE_RATE;
    }
    if (zero_valid != 0U && delta_sample_count > 0UL && delta_sample_count < ADC_MONITOR_CONTROL_MIN_WINDOW_SAMPLES)
    {
        next_state.invalid_reason_flags |= ADC_MONITOR_INVALID_WINDOW_TOO_SMALL;
    }
    if (zero_valid != 0U && delta_sample_count > ADC_MONITOR_CONTROL_MAX_WINDOW_SAMPLES)
    {
        next_state.invalid_reason_flags |= ADC_MONITOR_INVALID_SAMPLE_RATE;
    }
    next_current_valid =
        (samples_ready != 0U && zero_valid != 0U && raw_sample_count != 0UL && delta_sample_count != 0UL
         && dma_error_latched == 0U && ADC_MONITOR_CALIBRATION_ENABLED != 0U && MOTOR_CURRENT_VOLTS_PER_AMP > 0.0f)
            ? 1U
            : 0U;
    next_state.current_valid   = next_current_valid;
    next_current_control_valid = next_current_valid;

    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        static float    mean_current;
        static float    rms_current;
        static float    peak_current;
        static float    signed_mean_current;
        static float    noise_current;
        static float    trimmed_current;
        static uint32_t quality_flags;

        mean_current        = 0.0f;
        rms_current         = 0.0f;
        peak_current        = 0.0f;
        signed_mean_current = 0.0f;
        noise_current       = 0.0f;
        trimmed_current     = 0.0f;
        quality_flags       = 0UL;

        next_state.raw_current[i]           = raw_current[i];
        next_state.current_zero_raw[i]      = zero_raw[i];
        next_state.current_zero_span_raw[i] = zero_span_raw[i];
        if (zero_span_raw[i] > ADC_MONITOR_CURRENT_ZERO_MAX_SPAN_RAW)
        {
            quality_flags |= ADC_MONITOR_QUALITY_ZERO_UNSTABLE;
            next_state.invalid_reason_flags |= ADC_MONITOR_INVALID_ZERO_UNSTABLE;
        }
        if (delta_sample_count > 0UL && delta_sample_count < ADC_MONITOR_CONTROL_MIN_WINDOW_SAMPLES)
        {
            quality_flags |= ADC_MONITOR_QUALITY_WINDOW_TOO_SMALL;
        }
        if (delta_max[i] > ADC_MONITOR_CURRENT_SPIKE_MAX_RAW)
        {
            quality_flags |= ADC_MONITOR_QUALITY_WINDOW_SPIKE;
            next_state.invalid_reason_flags |= ADC_MONITOR_INVALID_WINDOW_SPIKE;
        }
        if (next_current_valid != 0U && delta_sample_count != 0UL)
        {
            AdcMonitor_ComputeCurrentWindow(delta_sum[i],
                                            delta_sq_sum[i],
                                            signed_delta_sum[i],
                                            signed_delta_sq_sum[i],
                                            delta_min[i],
                                            delta_max[i],
                                            delta_sample_count,
                                            (motor_id_t)i,
                                            &mean_current,
                                            &rms_current,
                                            &peak_current,
                                            &signed_mean_current,
                                            &noise_current,
                                            &trimmed_current);
            next_state.current_a[i]             = AdcMonitor_FilterCurrent(previous_current[i], trimmed_current);
            next_state.current_mean_a[i]        = mean_current;
            next_state.current_rms_a[i]         = rms_current;
            next_state.current_peak_a[i]        = peak_current;
            next_state.current_signed_mean_a[i] = signed_mean_current;
            next_state.current_noise_a[i]       = noise_current;
            next_state.current_sample_count[i]  = AdcMonitor_ClampU16(delta_sample_count);
        }
        else if (next_current_valid != 0U)
        {
            next_state.current_a[i] = previous_current[i];
        }
        else
        {
            next_state.current_a[i]             = 0.0f;
            next_state.current_signed_mean_a[i] = 0.0f;
            next_state.current_noise_a[i]       = 0.0f;
        }

        if (ChassisLayout_MotorEnabled((motor_id_t)i) == 0U)
        {
            next_state.raw_current[i]           = 0U;
            next_state.current_a[i]             = 0.0f;
            next_state.current_mean_a[i]        = 0.0f;
            next_state.current_rms_a[i]         = 0.0f;
            next_state.current_peak_a[i]        = 0.0f;
            next_state.current_signed_mean_a[i] = 0.0f;
            next_state.current_noise_a[i]       = 0.0f;
            next_state.current_sample_count[i]  = 0U;
            quality_flags                       = 0UL;
        }
        else if (ChassisLayout_MotorSide((motor_id_t)i) == MOTOR_SIDE_LEFT)
        {
            left_count++;
            left_raw_sum += next_state.raw_current[i];
            left_current_sum += next_state.current_a[i];
        }
        else
        {
            right_count++;
            right_raw_sum += next_state.raw_current[i];
            right_current_sum += next_state.current_a[i];
        }
        next_state.current_quality_flags[i] = quality_flags;
        if (quality_flags != 0UL)
        {
            next_current_control_valid = 0U;
        }
        if (next_current_valid != 0U && quality_flags == 0UL && ChassisLayout_MotorEnabled((motor_id_t)i) != 0U)
        {
            next_state.current_control_valid_mask |= (uint8_t)(1U << i);
        }
    }
    if (delta_sample_count < ADC_MONITOR_CONTROL_MIN_WINDOW_SAMPLES
        || delta_sample_count > ADC_MONITOR_CONTROL_MAX_WINDOW_SAMPLES)
    {
        next_current_control_valid            = 0U;
        next_state.current_control_valid_mask = 0U;
    }
    next_state.current_control_valid = next_current_control_valid;

    next_state.raw_left_current  = (left_count != 0U) ? (uint16_t)(left_raw_sum / left_count) : 0U;
    next_state.raw_right_current = (right_count != 0U) ? (uint16_t)(right_raw_sum / right_count) : 0U;
    next_state.left_current_a    = (left_count != 0U) ? (left_current_sum / (float)left_count) : 0.0f;
    next_state.right_current_a   = (right_count != 0U) ? (right_current_sum / (float)right_count) : 0.0f;

    primask = __get_PRIMASK();
    __disable_irq();
    adc_state                  = next_state;
    current_filter_initialized = next_current_valid;
    battery_filter_initialized = samples_ready;
    __set_PRIMASK(primask);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint8_t  zero_valid;
    uint32_t raw_count_before;
    uint32_t delta_count_before;

    if (hadc != &hadc1)
    {
        return;
    }

    for (uint8_t i = 0U; i < ADC_MONITOR_CHANNEL_COUNT; ++i)
    {
        adc_sample_snapshot[i] = adc_dma_buffer[i];
    }
    zero_valid         = current_zero_valid;
    raw_count_before   = adc_window_raw_count;
    delta_count_before = adc_window_delta_count;
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        uint16_t raw             = adc_dma_buffer[current_adc_index[i]];
        adc_window_latest_raw[i] = raw;
        adc_window_raw_sum[i] += raw;
        if (raw_count_before == 0UL || raw < adc_window_raw_min[i])
        {
            adc_window_raw_min[i] = raw;
        }
        if (raw_count_before == 0UL || raw > adc_window_raw_max[i])
        {
            adc_window_raw_max[i] = raw;
        }
        if (zero_valid != 0U)
        {
            uint16_t zero_raw         = current_zero_raw[i];
            uint16_t delta            = (raw >= zero_raw) ? (uint16_t)(raw - zero_raw) : 0U;
            int32_t  signed_delta     = (int32_t)raw - (int32_t)zero_raw;
            uint32_t signed_abs_delta = (signed_delta < 0) ? (uint32_t)(-signed_delta) : (uint32_t)signed_delta;
            adc_window_delta_sum[i] += delta;
            adc_window_delta_sq_sum[i] += (uint64_t)delta * (uint64_t)delta;
            adc_window_signed_delta_sum[i] += signed_delta;
            adc_window_signed_delta_sq_sum[i] += (uint64_t)signed_abs_delta * (uint64_t)signed_abs_delta;
            if (delta_count_before == 0UL)
            {
                adc_window_delta_min[i] = delta;
                adc_window_delta_max[i] = delta;
            }
            else
            {
                if (delta < adc_window_delta_min[i])
                {
                    adc_window_delta_min[i] = delta;
                }
                if (delta > adc_window_delta_max[i])
                {
                    adc_window_delta_max[i] = delta;
                }
            }
        }
    }
    adc_window_raw_count++;
    if (zero_valid != 0U)
    {
        adc_window_delta_count++;
    }
    adc_samples_ready = 1U;
}

void AdcMonitor_RequestCurrentZeroCalibration(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    current_zero_valid         = 0U;
    current_zero_sample_count  = 0U;
    current_zero_stationary    = 0U;
    current_filter_initialized = 0U;
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        current_zero_raw[i]                = 0U;
        current_zero_min_raw[i]            = 0U;
        current_zero_max_raw[i]            = 0U;
        current_zero_span_raw[i]           = 0U;
        current_zero_sum[i]                = 0UL;
        adc_state.current_a[i]             = 0.0f;
        adc_state.current_mean_a[i]        = 0.0f;
        adc_state.current_rms_a[i]         = 0.0f;
        adc_state.current_peak_a[i]        = 0.0f;
        adc_state.current_signed_mean_a[i] = 0.0f;
        adc_state.current_noise_a[i]       = 0.0f;
        adc_state.current_zero_raw[i]      = 0U;
        adc_state.current_zero_span_raw[i] = 0U;
        adc_state.current_sample_count[i]  = 0U;
        adc_state.current_quality_flags[i] = 0UL;
    }
    adc_state.current_zero_valid         = 0U;
    adc_state.current_zero_sample_count  = 0U;
    adc_state.current_valid              = 0U;
    adc_state.current_control_valid      = 0U;
    adc_state.current_control_valid_mask = 0U;
    AdcMonitor_ResetWindowAccumulators();
    __set_PRIMASK(primask);
}

void AdcMonitor_ApplyCurrentZeroCalibration(const uint16_t zero_raw[MOTOR_ID_COUNT])
{
    uint32_t primask;

    if (zero_raw == 0)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    current_zero_valid         = 1U;
    current_zero_sample_count  = ADC_MONITOR_CURRENT_ZERO_SAMPLES;
    current_filter_initialized = 0U;
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        current_zero_raw[i]                = zero_raw[i];
        current_zero_min_raw[i]            = zero_raw[i];
        current_zero_max_raw[i]            = zero_raw[i];
        current_zero_span_raw[i]           = 0U;
        current_zero_sum[i]                = (uint32_t)zero_raw[i] * ADC_MONITOR_CURRENT_ZERO_SAMPLES;
        adc_state.current_zero_raw[i]      = zero_raw[i];
        adc_state.current_zero_span_raw[i] = 0U;
    }
    adc_state.current_zero_valid        = 1U;
    adc_state.current_zero_sample_count = ADC_MONITOR_CURRENT_ZERO_SAMPLES;
    adc_state.current_valid             = (adc_state.samples_ready != 0U) ? 1U : 0U;
    __set_PRIMASK(primask);
}

void AdcMonitor_SetCurrentZeroStationary(uint8_t stationary)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    current_zero_stationary = (stationary != 0U) ? 1U : 0U;
    if (current_zero_valid == 0U && current_zero_stationary == 0U)
    {
        AdcMonitor_ResetCurrentZeroAccumulators();
        AdcMonitor_ResetWindowAccumulators();
        adc_state.current_zero_sample_count = 0U;
        adc_state.current_zero_valid        = 0U;
        for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
        {
            adc_state.current_zero_raw[i]      = 0U;
            adc_state.current_zero_span_raw[i] = 0U;
        }
    }
    __set_PRIMASK(primask);
}

void AdcMonitor_GetState(adc_monitor_state_t *state)
{
    uint32_t primask;

    if (state == 0)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *state = adc_state;
    __set_PRIMASK(primask);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        adc_dma_error_latched = 1U;
    }
}
