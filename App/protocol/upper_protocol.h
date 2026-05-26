#ifndef UPPER_PROTOCOL_H
#define UPPER_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UPPER_PROTOCOL_HEAD_0 0xA5U
#define UPPER_PROTOCOL_HEAD_1 0x5AU
#define UPPER_PROTOCOL_MAX_PAYLOAD 64U
#define UPPER_PROTOCOL_MAX_FRAME (UPPER_PROTOCOL_MAX_PAYLOAD + 5U)
#define UPPER_PROTOCOL_CMD_LEN(payload_len) ((uint8_t)(1U + (payload_len)))
#define UPPER_PROTOCOL_STATUS_PAYLOAD_LEN 45U
#define UPPER_PROTOCOL_VELOCITY_PAYLOAD_LEN 10U
#define UPPER_PROTOCOL_ESTOP_PAYLOAD_LEN 1U

typedef enum
{
  UPPER_CMD_SET_VELOCITY = 0x01,
  UPPER_CMD_ESTOP = 0x02,
  UPPER_CMD_STATUS = 0x81
} upper_protocol_cmd_t;

typedef struct
{
  float linear_x;
  float angular_z;
  uint8_t enable;
  uint8_t mode;
} upper_velocity_payload_t;

typedef struct
{
  float left_speed;
  float right_speed;
  int32_t left_encoder;
  int32_t right_encoder;
  float battery_voltage;
  float left_current;
  float right_current;
  int16_t imu_accel[3];
  int16_t imu_gyro[3];
  uint32_t error_flags;
  uint8_t control_mode;
} upper_status_payload_t;

uint8_t UpperProtocol_Checksum8(const uint8_t *data, uint16_t length);
uint16_t UpperProtocol_BuildFrame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len, uint8_t *out, uint16_t out_len);
uint8_t UpperProtocol_ParseVelocityPayload(const uint8_t *payload, uint8_t payload_len, upper_velocity_payload_t *velocity);
uint8_t UpperProtocol_BuildStatusPayload(const upper_status_payload_t *status, uint8_t *out, uint8_t out_len);

#ifdef __cplusplus
}
#endif

#endif
