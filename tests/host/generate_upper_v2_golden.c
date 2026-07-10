#include "upper_protocol.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

static void print_hex(const uint8_t *data, uint16_t length)
{
  for (uint16_t i = 0U; i < length; ++i)
  {
    (void)printf("%02x", data[i]);
  }
}

static void print_frame(const char *name,
                        uint8_t command,
                        const uint8_t *payload,
                        uint8_t payload_length,
                        uint8_t is_last)
{
  uint8_t frame[UPPER_PROTOCOL_MAX_FRAME] = {0U};
  uint16_t frame_length = UpperProtocol_BuildFrame(command,
                                                    payload,
                                                    payload_length,
                                                    frame,
                                                    sizeof(frame));

  (void)printf("    {\"name\":\"%s\",\"command\":%u,\"payload_length\":%u,"
               "\"frame_length\":%u,\"frame_hex\":\"",
               name,
               command,
               payload_length,
               frame_length);
  print_hex(frame, frame_length);
  (void)printf("\"}%s\n", is_last != 0U ? "" : ",");
}

static void emit_status(void)
{
  upper_status_payload_t status = {0};
  uint8_t payload[UPPER_PROTOCOL_STATUS_PAYLOAD_LEN] = {0U};

  status.battery_voltage = 12.345f;
  status.motor_speed_mps[0] = 1.25f;
  status.motor_speed_mps[1] = -0.5f;
  status.motor_speed_mps[2] = 0.001f;
  status.motor_speed_mps[3] = -2.0f;
  status.encoder_count[0] = 1;
  status.encoder_count[1] = -2;
  status.encoder_count[2] = INT32_MAX;
  status.encoder_count[3] = INT32_MIN;
  status.motor_current_a[0] = 0.1f;
  status.motor_current_a[1] = 1.2f;
  status.motor_current_a[2] = 2.4f;
  status.motor_current_a[3] = 65.535f;
  status.motor_target_mps[0] = 0.25f;
  status.motor_target_mps[1] = -0.25f;
  status.motor_target_mps[2] = 1.5f;
  status.motor_target_mps[3] = -1.5f;
  status.motor_output_permille[0] = 100;
  status.motor_output_permille[1] = -100;
  status.motor_output_permille[2] = 1000;
  status.motor_output_permille[3] = -1000;
  status.error_flags = 0x12345678UL;
  status.latched_error_flags = 0x90ABCDEFUL;
  status.status_flags = 0x0FU;
  status.control_source = 3U;
  status.motor_enabled_mask = 0x0FU;
  status.motor_speed_valid_mask = 0x05U;
  status.encoder_anomaly_mask = 0x0AU;
  status.comm_health_flags = 0x15U;

  (void)UpperProtocol_BuildStatusPayload(&status, payload, sizeof(payload));
  print_frame("status_v2", UPPER_CMD_STATUS, payload, sizeof(payload), 0U);
}

static void emit_imu(const char *name, int8_t temperature_c, uint8_t is_last)
{
  upper_imu_status_payload_t imu = {0};
  uint8_t payload[UPPER_PROTOCOL_IMU_STATUS_PAYLOAD_LEN] = {0U};

  imu.accel_g[0] = 0.125f;
  imu.accel_g[1] = -0.25f;
  imu.accel_g[2] = 1.0f;
  imu.gyro_corrected_dps[0] = 1.5f;
  imu.gyro_corrected_dps[1] = -2.5f;
  imu.gyro_corrected_dps[2] = 3.5f;
  imu.euler_deg[0] = 10.0f;
  imu.euler_deg[1] = -20.0f;
  imu.euler_deg[2] = 30.0f;
  imu.quaternion[0] = 1.0f;
  imu.quaternion[1] = 0.0f;
  imu.quaternion[2] = 0.0f;
  imu.quaternion[3] = 0.0f;
  imu.timestamp_ms = 0x01020304UL;
  imu.sensor_time = 0x05060708UL;
  imu.sample_count = 0x11121314UL;
  imu.quality_flags = 0x21222324UL;
  for (uint8_t i = 0U; i < 7U; ++i)
  {
    imu.quality_counters[i] = (uint32_t)i + 1UL;
  }
  imu.status_flags = 0x0BU;
  imu.temperature_c = temperature_c;

  (void)UpperProtocol_BuildImuStatusPayload(&imu, payload, sizeof(payload));
  print_frame(name, UPPER_CMD_IMU_STATUS, payload, sizeof(payload), is_last);
}

int main(void)
{
  (void)printf("{\n  \"schema\":1,\n  \"protocol_version\":2,\n  \"frames\":[\n");
  emit_status();
  emit_imu("imu_temp_23c", 23, 0U);
  emit_imu("imu_temp_minus_41c", -41, 0U);
  emit_imu("imu_temp_87c", 87, 1U);
  (void)printf("  ]\n}\n");
  return 0;
}
