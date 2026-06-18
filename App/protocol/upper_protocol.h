#ifndef UPPER_PROTOCOL_H
#define UPPER_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UPPER_PROTOCOL_HEAD_0 0xA5U
#define UPPER_PROTOCOL_HEAD_1 0x5AU
#define UPPER_PROTOCOL_VERSION 2U
#define UPPER_PROTOCOL_MAX_PAYLOAD 64U
#define UPPER_PROTOCOL_MAX_FRAME (UPPER_PROTOCOL_MAX_PAYLOAD + 5U)
#define UPPER_PROTOCOL_CMD_LEN(payload_len) ((uint8_t)(1U + (payload_len)))
#define UPPER_PROTOCOL_STATUS_PAYLOAD_LEN 64U
#define UPPER_PROTOCOL_VELOCITY_PAYLOAD_LEN 10U
#define UPPER_PROTOCOL_ESTOP_PAYLOAD_LEN 1U
#define UPPER_PROTOCOL_LINE_CTRL_PAYLOAD_LEN 1U
#define UPPER_PROTOCOL_MOTOR_COUNT 4U

#define UPPER_STATUS_FLAG_ESTOP           (1U << 0)
#define UPPER_STATUS_FLAG_FAULT_STOP      (1U << 1)
#define UPPER_STATUS_FLAG_LINE_ENABLED    (1U << 2)
#define UPPER_STATUS_FLAG_SPEED_VALID_ALL (1U << 3)

typedef enum
{
  UPPER_CMD_SET_VELOCITY = 0x01,
  UPPER_CMD_ESTOP = 0x02,
  UPPER_CMD_LINE_CTRL = 0x03,
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
  float battery_voltage;
  float motor_speed_mps[UPPER_PROTOCOL_MOTOR_COUNT];
  int32_t encoder_count[UPPER_PROTOCOL_MOTOR_COUNT];
  float motor_current_a[UPPER_PROTOCOL_MOTOR_COUNT];
  float motor_target_mps[UPPER_PROTOCOL_MOTOR_COUNT];
  int16_t motor_output_permille[UPPER_PROTOCOL_MOTOR_COUNT];
  uint32_t error_flags;
  uint32_t latched_error_flags;
  uint8_t status_flags;
  uint8_t control_source;
  uint8_t motor_enabled_mask;
  uint8_t motor_speed_valid_mask;
} upper_status_payload_t;

uint8_t UpperProtocol_Checksum8(const uint8_t *data, uint16_t length);
uint16_t UpperProtocol_BuildFrame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len, uint8_t *out, uint16_t out_len);
uint8_t UpperProtocol_ParseVelocityPayload(const uint8_t *payload, uint8_t payload_len, upper_velocity_payload_t *velocity);
uint8_t UpperProtocol_BuildStatusPayload(const upper_status_payload_t *status, uint8_t *out, uint8_t out_len);

#ifdef __cplusplus
}
#endif

#endif
