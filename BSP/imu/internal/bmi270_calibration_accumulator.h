#ifndef BMI270_CALIBRATION_ACCUMULATOR_H
#define BMI270_CALIBRATION_ACCUMULATOR_H

#include <stdint.h>

typedef enum
{
    BMI270_CALIBRATION_ACC_IDLE       = 0,
    BMI270_CALIBRATION_ACC_COLLECTING = 1,
    BMI270_CALIBRATION_ACC_READY      = 2,
    BMI270_CALIBRATION_ACC_FAIL_ABS   = 3,
    BMI270_CALIBRATION_ACC_FAIL_SPAN  = 4
} bmi270_calibration_acc_state_t;

typedef struct
{
    float    sum_dps[3];
    float    accel_sum_g[3];
    float    min_dps[3];
    float    max_dps[3];
    uint32_t last_sample_ms;
    uint16_t target_samples;
    uint16_t sample_count;
    uint16_t interval_ms;
    uint8_t  state;
    uint8_t  fail_axis;
    uint8_t  has_last_sample;
} bmi270_calibration_accumulator_t;

void                           Bmi270CalibrationAccumulator_Init(bmi270_calibration_accumulator_t *accumulator);
uint8_t                        Bmi270CalibrationAccumulator_Begin(bmi270_calibration_accumulator_t *accumulator,
                                                                  uint16_t                          target_samples,
                                                                  uint16_t                          interval_ms);
void                           Bmi270CalibrationAccumulator_Restart(bmi270_calibration_accumulator_t *accumulator);
bmi270_calibration_acc_state_t Bmi270CalibrationAccumulator_Feed(bmi270_calibration_accumulator_t *accumulator,
                                                                 uint32_t                          now_ms,
                                                                 const float                       accel_g[3],
                                                                 const float                       gyro_dps[3],
                                                                 float                             max_abs_dps,
                                                                 float                             max_span_dps);
uint8_t Bmi270CalibrationAccumulator_GetResult(const bmi270_calibration_accumulator_t *accumulator,
                                               float                                   bias_dps[3],
                                               float                                   accel_mean_g[3]);

#endif /* BMI270_CALIBRATION_ACCUMULATOR_H */
