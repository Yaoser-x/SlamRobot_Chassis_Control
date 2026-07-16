#ifndef ENCODER_CONFIG_H
#define ENCODER_CONFIG_H

/** Encoder electrical and mechanical conversion constants. */
#define CHASSIS_ENCODER_BASE_PPR                  11.0f
#define CHASSIS_ENCODER_QUADRATURE_MULT           4.0f
#define CHASSIS_MOTOR_GEAR_RATIO                  56.0f
#define CHASSIS_MIN_ENCODER_DT_MS                 1U
#define CHASSIS_MAX_ENCODER_DT_MS                 100U
#define CHASSIS_ENCODER_MAX_ABS_MPS               2.5f
#define CHASSIS_ENCODER_SPIKE_REJECT_MPS          0.45f
#define CHASSIS_ENCODER_FILTER_MIN_SAMPLES        3U
#define CHASSIS_ENCODER_REBUILD_REJECTS           3U
#define CHASSIS_ENCODER_MAX_CONSECUTIVE_ANOMALIES 10U
#define CHASSIS_ENCODER_SIDE_SPEED_DIFF_MPS       0.25f
#define CHASSIS_ENCODER_SIDE_COUNT_DIFF           1000

#endif
