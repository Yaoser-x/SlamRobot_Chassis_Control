#include "upper_protocol.h"

#include <string.h>

static void UpperProtocol_WriteU32(uint8_t *out, uint32_t value)
{
  out[0] = (uint8_t)(value & 0xFFU);
  out[1] = (uint8_t)((value >> 8) & 0xFFU);
  out[2] = (uint8_t)((value >> 16) & 0xFFU);
  out[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static void UpperProtocol_WriteU16(uint8_t *out, uint16_t value)
{
  out[0] = (uint8_t)(value & 0xFFU);
  out[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void UpperProtocol_WriteI32(uint8_t *out, int32_t value)
{
  UpperProtocol_WriteU32(out, (uint32_t)value);
}

static void UpperProtocol_WriteI16(uint8_t *out, int16_t value)
{
  uint16_t raw = (uint16_t)value;
  out[0] = (uint8_t)(raw & 0xFFU);
  out[1] = (uint8_t)((raw >> 8) & 0xFFU);
}

static int32_t UpperProtocol_RoundScaledFloat(float value, float scale)
{
  float scaled = value * scale;

  if (scaled >= 0.0f)
  {
    return (int32_t)(scaled + 0.5f);
  }
  return (int32_t)(scaled - 0.5f);
}

static uint16_t UpperProtocol_SaturateU16FromFloat(float value, float scale)
{
  int32_t rounded = UpperProtocol_RoundScaledFloat(value, scale);

  if (rounded <= 0)
  {
    return 0U;
  }
  if (rounded >= 65535)
  {
    return 65535U;
  }
  return (uint16_t)rounded;
}

static int16_t UpperProtocol_SaturateI16FromFloat(float value, float scale)
{
  int32_t rounded = UpperProtocol_RoundScaledFloat(value, scale);

  if (rounded > 32767)
  {
    return 32767;
  }
  if (rounded < -32768)
  {
    return -32768;
  }
  return (int16_t)rounded;
}

static int16_t UpperProtocol_SaturateI16(int16_t value)
{
  return value;
}

static uint32_t UpperProtocol_ReadU32(const uint8_t *in)
{
  return ((uint32_t)in[0]) |
         ((uint32_t)in[1] << 8) |
         ((uint32_t)in[2] << 16) |
         ((uint32_t)in[3] << 24);
}

static float UpperProtocol_ReadFloat(const uint8_t *in)
{
  uint32_t raw = UpperProtocol_ReadU32(in);
  float value = 0.0f;
  (void)memcpy(&value, &raw, sizeof(value));
  return value;
}

uint8_t UpperProtocol_Checksum8(const uint8_t *data, uint16_t length)
{
  static const uint8_t crc8_table[256] = {
    0x00U, 0x5EU, 0xBCU, 0xE2U, 0x61U, 0x3FU, 0xDDU, 0x83U,
    0xC2U, 0x9CU, 0x7EU, 0x20U, 0xA3U, 0xFDU, 0x1FU, 0x41U,
    0x9DU, 0xC3U, 0x21U, 0x7FU, 0xFCU, 0xA2U, 0x40U, 0x1EU,
    0x5FU, 0x01U, 0xE3U, 0xBDU, 0x3EU, 0x60U, 0x82U, 0xDCU,
    0x23U, 0x7DU, 0x9FU, 0xC1U, 0x42U, 0x1CU, 0xFEU, 0xA0U,
    0xE1U, 0xBFU, 0x5DU, 0x03U, 0x80U, 0xDEU, 0x3CU, 0x62U,
    0xBEU, 0xE0U, 0x02U, 0x5CU, 0xDFU, 0x81U, 0x63U, 0x3DU,
    0x7CU, 0x22U, 0xC0U, 0x9EU, 0x1DU, 0x43U, 0xA1U, 0xFFU,
    0x46U, 0x18U, 0xFAU, 0xA4U, 0x27U, 0x79U, 0x9BU, 0xC5U,
    0x84U, 0xDAU, 0x38U, 0x66U, 0xE5U, 0xBBU, 0x59U, 0x07U,
    0xDBU, 0x85U, 0x67U, 0x39U, 0xBAU, 0xE4U, 0x06U, 0x58U,
    0x19U, 0x47U, 0xA5U, 0xFBU, 0x78U, 0x26U, 0xC4U, 0x9AU,
    0x65U, 0x3BU, 0xD9U, 0x87U, 0x04U, 0x5AU, 0xB8U, 0xE6U,
    0xA7U, 0xF9U, 0x1BU, 0x45U, 0xC6U, 0x98U, 0x7AU, 0x24U,
    0xF8U, 0xA6U, 0x44U, 0x1AU, 0x99U, 0xC7U, 0x25U, 0x7BU,
    0x3AU, 0x64U, 0x86U, 0xD8U, 0x5BU, 0x05U, 0xE7U, 0xB9U,
    0x8CU, 0xD2U, 0x30U, 0x6EU, 0xEDU, 0xB3U, 0x51U, 0x0FU,
    0x4EU, 0x10U, 0xF2U, 0xACU, 0x2FU, 0x71U, 0x93U, 0xCDU,
    0x11U, 0x4FU, 0xADU, 0xF3U, 0x70U, 0x2EU, 0xCCU, 0x92U,
    0xD3U, 0x8DU, 0x6FU, 0x31U, 0xB2U, 0xECU, 0x0EU, 0x50U,
    0xAFU, 0xF1U, 0x13U, 0x4DU, 0xCEU, 0x90U, 0x72U, 0x2CU,
    0x6DU, 0x33U, 0xD1U, 0x8FU, 0x0CU, 0x52U, 0xB0U, 0xEEU,
    0x32U, 0x6CU, 0x8EU, 0xD0U, 0x53U, 0x0DU, 0xEFU, 0xB1U,
    0xF0U, 0xAEU, 0x4CU, 0x12U, 0x91U, 0xCFU, 0x2DU, 0x73U,
    0xCAU, 0x94U, 0x76U, 0x28U, 0xABU, 0xF5U, 0x17U, 0x49U,
    0x08U, 0x56U, 0xB4U, 0xEAU, 0x69U, 0x37U, 0xD5U, 0x8BU,
    0x57U, 0x09U, 0xEBU, 0xB5U, 0x36U, 0x68U, 0x8AU, 0xD4U,
    0x95U, 0xCBU, 0x29U, 0x77U, 0xF4U, 0xAAU, 0x48U, 0x16U,
    0xE9U, 0xB7U, 0x55U, 0x0BU, 0x88U, 0xD6U, 0x34U, 0x6AU,
    0x2BU, 0x75U, 0x97U, 0xC9U, 0x4AU, 0x14U, 0xF6U, 0xA8U,
    0x74U, 0x2AU, 0xC8U, 0x96U, 0x15U, 0x4BU, 0xA9U, 0xF7U,
    0xB6U, 0xE8U, 0x0AU, 0x54U, 0xD7U, 0x89U, 0x6BU, 0x35U,
  };
  uint8_t crc = 0U;

  if (data == 0)
  {
    return 0U;
  }

  for (uint16_t i = 0U; i < length; ++i)
  {
    crc = crc8_table[crc ^ data[i]];
  }

  return crc;
}

uint16_t UpperProtocol_BuildFrame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len, uint8_t *out, uint16_t out_len)
{
  uint8_t cmd_len = UPPER_PROTOCOL_CMD_LEN(payload_len);
  uint16_t frame_len = (uint16_t)cmd_len + 4U;

  if (out == 0 || out_len < frame_len || payload_len > UPPER_PROTOCOL_MAX_PAYLOAD)
  {
    return 0U;
  }
  if (payload_len > 0U && payload == 0)
  {
    return 0U;
  }

  out[0] = UPPER_PROTOCOL_HEAD_0;
  out[1] = UPPER_PROTOCOL_HEAD_1;
  out[2] = cmd_len;
  out[3] = cmd;
  if (payload_len > 0U)
  {
    (void)memcpy(&out[4], payload, payload_len);
  }
  out[frame_len - 1U] = UpperProtocol_Checksum8(&out[2], (uint16_t)cmd_len + 1U);
  return frame_len;
}

uint8_t UpperProtocol_ParseVelocityPayload(const uint8_t *payload, uint8_t payload_len, upper_velocity_payload_t *velocity)
{
  if (payload == 0 || velocity == 0 || payload_len != UPPER_PROTOCOL_VELOCITY_PAYLOAD_LEN)
  {
    return 0U;
  }

  velocity->linear_x = UpperProtocol_ReadFloat(&payload[0]);
  velocity->angular_z = UpperProtocol_ReadFloat(&payload[4]);
  velocity->enable = payload[8];
  velocity->mode = payload[9];
  return 1U;
}

uint8_t UpperProtocol_BuildStatusPayload(const upper_status_payload_t *status, uint8_t *out, uint8_t out_len)
{
  uint8_t offset = 0U;

  if (status == 0 || out == 0 || out_len < UPPER_PROTOCOL_STATUS_PAYLOAD_LEN)
  {
    return 0U;
  }

  out[offset++] = UPPER_PROTOCOL_VERSION;
  out[offset++] = status->status_flags;
  out[offset++] = status->control_source;
  out[offset++] = status->motor_enabled_mask;

  UpperProtocol_WriteU32(&out[offset], status->error_flags);
  offset = (uint8_t)(offset + 4U);
  UpperProtocol_WriteU32(&out[offset], status->latched_error_flags);
  offset = (uint8_t)(offset + 4U);
  UpperProtocol_WriteU16(&out[offset], UpperProtocol_SaturateU16FromFloat(status->battery_voltage, 1000.0f));
  offset = (uint8_t)(offset + 2U);

  for (uint8_t i = 0U; i < UPPER_PROTOCOL_MOTOR_COUNT; ++i)
  {
    UpperProtocol_WriteI16(&out[offset], UpperProtocol_SaturateI16FromFloat(status->motor_speed_mps[i], 1000.0f));
    offset = (uint8_t)(offset + 2U);
  }
  for (uint8_t i = 0U; i < UPPER_PROTOCOL_MOTOR_COUNT; ++i)
  {
    UpperProtocol_WriteI32(&out[offset], status->encoder_count[i]);
    offset = (uint8_t)(offset + 4U);
  }
  for (uint8_t i = 0U; i < UPPER_PROTOCOL_MOTOR_COUNT; ++i)
  {
    UpperProtocol_WriteU16(&out[offset], UpperProtocol_SaturateU16FromFloat(status->motor_current_a[i], 1000.0f));
    offset = (uint8_t)(offset + 2U);
  }
  for (uint8_t i = 0U; i < UPPER_PROTOCOL_MOTOR_COUNT; ++i)
  {
    UpperProtocol_WriteI16(&out[offset], UpperProtocol_SaturateI16FromFloat(status->motor_target_mps[i], 1000.0f));
    offset = (uint8_t)(offset + 2U);
  }
  for (uint8_t i = 0U; i < UPPER_PROTOCOL_MOTOR_COUNT; ++i)
  {
    UpperProtocol_WriteI16(&out[offset], UpperProtocol_SaturateI16(status->motor_output_permille[i]));
    offset = (uint8_t)(offset + 2U);
  }
  out[offset++] = status->motor_speed_valid_mask;
  out[offset++] = status->encoder_anomaly_mask;
  out[offset] = status->comm_health_flags;

  return UPPER_PROTOCOL_STATUS_PAYLOAD_LEN;
}

static void UpperProtocol_WriteFloat(uint8_t *out, float value)
{
  uint32_t raw;
  (void)memcpy(&raw, &value, sizeof(raw));
  UpperProtocol_WriteU32(out, raw);
}

uint8_t UpperProtocol_BuildImuStatusPayload(const upper_imu_status_payload_t *imu, uint8_t *out, uint8_t out_len)
{
  uint8_t offset = 0U;

  if (imu == 0 || out == 0 || out_len < UPPER_PROTOCOL_IMU_STATUS_PAYLOAD_LEN)
  {
    return 0U;
  }

  out[offset++] = UPPER_PROTOCOL_VERSION;

  for (uint8_t i = 0U; i < 3U; ++i)
  {
    UpperProtocol_WriteFloat(&out[offset], imu->accel_g[i]);
    offset = (uint8_t)(offset + 4U);
  }
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    UpperProtocol_WriteFloat(&out[offset], imu->gyro_corrected_dps[i]);
    offset = (uint8_t)(offset + 4U);
  }
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    UpperProtocol_WriteFloat(&out[offset], imu->euler_deg[i]);
    offset = (uint8_t)(offset + 4U);
  }

  UpperProtocol_WriteU32(&out[offset], imu->timestamp_ms);
  offset = (uint8_t)(offset + 4U);

  out[offset++] = imu->status_flags;
  out[offset] = (uint8_t)((int8_t)imu->temperature_c);

  return UPPER_PROTOCOL_IMU_STATUS_PAYLOAD_LEN;
}
