#include "imu_bmi270_calibration.h"
#include "imu_bmi270_fifo.h"
#include "imu_bmi270.h"
#include "imu_bmi270_math.h"
#include "imu_bmi270_profile.h"
#include "imu_bmi270_time.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures;

static void require_int(int condition, const char *message)
{
  if (!condition)
  {
    printf("FAIL: %s\n", message);
    failures++;
  }
}

static void require_close(float actual, float expected, float tolerance, const char *message)
{
  if (fabsf(actual - expected) > tolerance)
  {
    printf("FAIL: %s actual=%f expected=%f tol=%f\n", message, actual, expected, tolerance);
    failures++;
  }
}

static void test_fifo_header_sensor_time_and_overflow(void)
{
  const uint8_t fifo[] = {
    0x8CU,
    0x04U, 0x00U, 0x05U, 0x00U, 0x06U, 0x00U,
    0x01U, 0x00U, 0x02U, 0x00U, 0x03U, 0x00U,
    0x44U, 0x10U, 0x20U, 0x30U,
    0x40U, 0x02U,
    0x80U
  };
  imu_bmi270_fifo_parse_result_t result;
  imu_bmi270_fifo_sample_t samples[2];

  memset(&result, 0, sizeof(result));
  memset(samples, 0, sizeof(samples));

  require_int(ImuBmi270Fifo_Parse(fifo, sizeof(fifo), samples, 2U, &result) == 1U,
              "fifo parser accepts regular, sensortime, skip, overread frames");
  require_int(result.sample_count == 1U, "fifo parser returns one sample");
  require_int(samples[0].accel_raw[0] == 1 && samples[0].accel_raw[2] == 3,
              "fifo parser decodes accel after gyro in combined frame");
  require_int(samples[0].gyro_raw[0] == 4 && samples[0].gyro_raw[2] == 6,
              "fifo parser decodes gyro first in combined frame");
  require_int(result.sensor_time_valid == 1U && result.sensor_time == 0x302010UL,
              "fifo parser decodes 24-bit sensor time");
  require_int((result.flags & IMU_BMI270_FIFO_PARSE_SKIP_FRAME) != 0U,
              "fifo parser flags skip frame overflow evidence");
  require_int(result.skipped_frame_count == 2U, "fifo parser counts skipped frames");
  require_int((result.flags & IMU_BMI270_FIFO_PARSE_OVERREAD) != 0U,
              "fifo parser flags overread frame");
}

static void test_sensor_time_delta_and_validation(void)
{
  float dt_s = 0.0f;

  require_int(ImuBmi270Time_DeltaTicks24(0x000020UL, 0x000010UL) == 0x10UL,
              "sensor time delta handles normal increments");
  require_int(ImuBmi270Time_DeltaTicks24(0x000005UL, 0xFFFFF0UL) == 0x15UL,
              "sensor time delta handles 24-bit wrap");
  require_int(ImuBmi270Time_DeltaSeconds(0x000110UL, 0x000010UL, 0.02f, &dt_s) == 1U,
              "sensor time accepts bounded dt");
  require_close(dt_s, 0.010f, 0.0001f, "sensor time converts ticks to seconds");
  require_int(ImuBmi270Time_DeltaSeconds(0x000010UL, 0x000010UL, 0.02f, &dt_s) == 0U,
              "sensor time rejects zero dt");
  require_int(ImuBmi270Time_DeltaSeconds(0x010010UL, 0x000010UL, 0.02f, &dt_s) == 0U,
              "sensor time rejects abnormal dt");
}

static void test_raw_frame_signal_guard_rejects_all_zero_frame(void)
{
  const int16_t zero_accel[3] = {0, 0, 0};
  const int16_t zero_gyro[3] = {0, 0, 0};
  const int16_t accel_with_signal[3] = {0, 0, 1};
  const int16_t gyro_with_signal[3] = {0, -1, 0};

  require_int(ImuBmi270_RawFrameHasSignal(zero_accel, zero_gyro) == 0U,
              "raw frame guard rejects all-zero accel and gyro");
  require_int(ImuBmi270_RawFrameHasSignal(accel_with_signal, zero_gyro) == 1U,
              "raw frame guard accepts nonzero accel");
  require_int(ImuBmi270_RawFrameHasSignal(zero_accel, gyro_with_signal) == 1U,
              "raw frame guard accepts nonzero gyro");
  require_int(ImuBmi270_RawFrameHasSignal(0, zero_gyro) == 0U,
              "raw frame guard rejects null accel pointer");
  require_int(ImuBmi270_RawFrameHasSignal(zero_accel, 0) == 0U,
              "raw frame guard rejects null gyro pointer");
}

static void test_gyro_cal_span_limit_reports_first_unstable_axis(void)
{
  const float stable_min[3] = {-1.0f, -2.0f, -0.5f};
  const float stable_max[3] = {1.0f, 2.0f, 0.5f};
  const float unstable_min[3] = {-1.0f, -2.0f, -0.5f};
  const float unstable_max[3] = {1.0f, 3.5f, 7.0f};
  uint8_t axis = 99U;

  require_int(ImuBmi270_GyroCalSpanWithinLimit(stable_min, stable_max, 5.0f, &axis) == 1U,
              "gyro cal span accepts axes at or below limit");
  require_int(ImuBmi270_GyroCalSpanWithinLimit(unstable_min, unstable_max, 5.0f, &axis) == 0U,
              "gyro cal span rejects first axis above limit");
  require_int(axis == 1U, "gyro cal span reports first unstable axis");
  require_int(ImuBmi270_GyroCalSpanWithinLimit(0, stable_max, 5.0f, &axis) == 0U,
              "gyro cal span rejects null min pointer");
}

static void test_auto_cal_scheduler_waits_for_online_uncalibrated_due_state(void)
{
  require_int(ImuBmi270_AutoCalDue(1U, 1U, 0U, 0U, 3U, 1000UL, 1000UL) == 1U,
              "auto cal runs when online uncalibrated and due");
  require_int(ImuBmi270_AutoCalDue(1U, 1U, 0U, 0U, 3U, 999UL, 1000UL) == 0U,
              "auto cal waits until due time");
  require_int(ImuBmi270_AutoCalDue(0U, 1U, 0U, 0U, 3U, 1000UL, 1000UL) == 0U,
              "auto cal respects disabled flag");
  require_int(ImuBmi270_AutoCalDue(1U, 0U, 0U, 0U, 3U, 1000UL, 1000UL) == 0U,
              "auto cal waits for online IMU");
  require_int(ImuBmi270_AutoCalDue(1U, 1U, 1U, 0U, 3U, 1000UL, 1000UL) == 0U,
              "auto cal skips already calibrated IMU");
  require_int(ImuBmi270_AutoCalDue(1U, 1U, 0U, 3U, 3U, 1000UL, 1000UL) == 0U,
              "auto cal stops at max attempts");
}

static void test_profile_register_values_are_named_and_checkable(void)
{
  const imu_bmi270_profile_t *profile = ImuBmi270Profile_Get(IMU_BMI270_PROFILE_PERFORMANCE);
  imu_bmi270_profile_check_t check;

  require_int(profile != 0, "performance profile exists");
  require_int(profile->acc_conf == 0xA8U, "performance profile keeps current accel ODR/bandwidth");
  require_int(profile->gyr_conf == 0xE8U, "performance profile keeps current gyro ODR/bandwidth");
  require_int(profile->fifo_config_1 == 0xD0U, "performance profile enables header acc gyro FIFO");
  check.acc_conf = profile->acc_conf;
  check.acc_range = profile->acc_range;
  check.gyr_conf = profile->gyr_conf;
  check.gyr_range = profile->gyr_range;
  check.pwr_conf = profile->pwr_conf;
  check.pwr_ctrl = profile->pwr_ctrl;
  check.fifo_config_0 = profile->fifo_config_0;
  check.fifo_config_1 = profile->fifo_config_1;
  check.int_map_data = profile->int_map_data;
  check.int1_io_ctrl = profile->int1_io_ctrl;
  require_int(ImuBmi270Profile_Check(profile, &check) == 1U, "profile readback succeeds when values match");
  check.gyr_range ^= 1U;
  require_int(ImuBmi270Profile_Check(profile, &check) == 0U, "profile readback fails on mismatch");
}

static void test_coordinate_mapping_and_calibration_defaults(void)
{
  imu_bmi270_calibration_t calibration;
  float sensor[3] = {1.0f, -2.0f, 3.0f};
  float body[3] = {0.0f, 0.0f, 0.0f};
  float ros[3] = {0.0f, 0.0f, 0.0f};

  ImuBmi270Calibration_Default(&calibration);
  require_int(calibration.version == IMU_BMI270_CALIBRATION_VERSION,
              "default calibration has version");
  require_int(ImuBmi270Calibration_Validate(&calibration) == 1U,
              "default calibration validates CRC");

  ImuBmi270Coordinate_Apply(calibration.sensor_to_body, sensor, body);
  require_close(body[0], 1.0f, 0.0001f, "default sensor to body x");
  require_close(body[1], -2.0f, 0.0001f, "default sensor to body y");
  require_close(body[2], 3.0f, 0.0001f, "default sensor to body z");
  ImuBmi270Coordinate_BodyToRos(body, ros);
  require_close(ros[0], body[0], 0.0001f, "ROS forward matches body x");
  require_close(ros[1], body[1], 0.0001f, "ROS left matches body y");
  require_close(ros[2], body[2], 0.0001f, "ROS up matches body z");
}

static void test_mahony_static_and_yaw_continuity(void)
{
  imu_bmi270_mahony_t fusion;
  imu_bmi270_mahony_params_t params = ImuBmi270Mahony_DefaultParams();
  float accel[3] = {0.0f, 0.0f, 1.0f};
  float gyro[3] = {0.0f, 0.0f, 90.0f};
  float euler[3] = {0.0f, 0.0f, 0.0f};

  ImuBmi270Mahony_Init(&fusion);
  for (uint32_t i = 0U; i < 100U; ++i)
  {
    ImuBmi270Mahony_Update(&fusion, gyro, accel, 0.01f, &params);
  }
  ImuBmi270Quaternion_ToEulerDeg(&fusion.q, euler);
  require_close(ImuBmi270Quaternion_Norm(&fusion.q), 1.0f, 0.001f,
                "mahony keeps quaternion normalized");
  require_close(euler[2], 90.0f, 2.0f, "mahony integrates yaw continuously");

  accel[2] = 3.0f;
  ImuBmi270Mahony_Update(&fusion, gyro, accel, 0.01f, &params);
  require_int((fusion.status_flags & IMU_BMI270_FUSION_ACCEL_DEGRADED) != 0U,
              "mahony degrades accel correction on abnormal acceleration norm");
}

static void test_quaternion_initializes_roll_pitch_from_accel(void)
{
  imu_bmi270_quaternion_t q;
  float accel[3] = {0.010f, -0.002f, 0.995f};
  float euler[3] = {0.0f, 0.0f, 0.0f};

  require_int(ImuBmi270Quaternion_FromAccel(accel, &q) == 1U,
              "quaternion initializes from nonzero accel");
  ImuBmi270Quaternion_ToEulerDeg(&q, euler);
  require_close(euler[0], -0.115f, 0.02f, "accel init sets roll from gravity");
  require_close(euler[1], -0.576f, 0.02f, "accel init sets pitch from gravity");
  require_close(euler[2], 0.0f, 0.02f, "accel init resets yaw reference");
  require_int(ImuBmi270Quaternion_FromAccel(0, &q) == 0U,
              "quaternion accel init rejects null accel");
}

int main(void)
{
  test_fifo_header_sensor_time_and_overflow();
  test_sensor_time_delta_and_validation();
  test_raw_frame_signal_guard_rejects_all_zero_frame();
  test_gyro_cal_span_limit_reports_first_unstable_axis();
  test_auto_cal_scheduler_waits_for_online_uncalibrated_due_state();
  test_profile_register_values_are_named_and_checkable();
  test_coordinate_mapping_and_calibration_defaults();
  test_mahony_static_and_yaw_continuity();
  test_quaternion_initializes_roll_pitch_from_accel();

  if (failures != 0)
  {
    printf("%d failures\n", failures);
    return 1;
  }
  printf("imu pipeline tests passed\n");
  return 0;
}
