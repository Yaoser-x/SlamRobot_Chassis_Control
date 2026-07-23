/*
 * F407_ESP12F.ino — ESP8266 WiFi 桥接固件（Arduino IDE）
 *
 * 功能：
 *   - 首次启动开放 F407_Chassis_Setup 设置密码，配置后创建 F407_Chassis
 *   - WebSocket 实时遥控 + 遥测回传
 *   - 手机网页操控界面（虚拟摇杆 + 速度/方向控制）
 *   - robot_link_protocol 帧协议与 STM32F407 USART2 通信
 *
 * 硬件：
 *   ESP12F (ESP8266)  UART0 @ 115200 8N1
 *   协议帧：0xA5 0x5A CMD_LEN CMD PAYLOAD(0..64B) CRC8
 *
 * 编译：Arduino IDE → Generic ESP8266 Module → 115200 → Flash Size 4MB
 * 烧录：通过 STM32 调试台 → espflash on → esptool.py
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <WebSocketsServer.h>
#include "esp_link_policy.h"
#include "esp_frame_parser.h"

#include "html_page_gz.h"

// ============================================================================
// 1. 配置常量
// ============================================================================

#define UART_BAUD         115200
#define AP_SSID           "F407_Chassis"
#define SETUP_AP_SSID     "F407_Chassis_Setup"
#define WEB_SOCKET_PORT   81
#define HTTP_PORT         80

#define CONFIG_VERSION          1U
#define CONFIG_PASSWORD_MIN_LEN 8U
#define CONFIG_PASSWORD_MAX_LEN 63U
#define CONFIG_CRC_OFFSET       69U
#define CONFIG_RECORD_SIZE      73U

static const uint8_t CONFIG_MAGIC[4] = {'F', '4', '0', '7'};
static char g_ap_password[CONFIG_PASSWORD_MAX_LEN + 1U] = {};
static bool g_config_valid = false;
static unsigned long g_reboot_at_ms = 0;

// 底盘控制范围
#define MAX_LINEAR_MPS    0.5f    // 最大线速度 m/s
#define MAX_ANGULAR_RPS   1.0f    // 最大角速度 rad/s

// 遥测推送间隔（ms）
#define TELEM_INTERVAL_MS 100

// 底盘状态（解析自 STM32 status 帧）
struct ChassisStatus {
  uint8_t  protocol_version;
  uint8_t  status_flags;
  uint8_t  control_source;
  uint8_t  motor_enabled_mask;
  uint32_t error_flags;
  uint32_t latched_error_flags;
  uint16_t battery_mv;
  int16_t  motor_speed_mmps[4];
  int32_t  encoder_count[4];
  uint16_t motor_current_ma[4];
  int16_t  motor_target_mmps[4];
  int16_t  motor_output_permille[4];
  uint8_t  motor_speed_valid_mask;
  uint8_t  encoder_anomaly_mask;
  uint8_t  comm_health_flags;
  uint32_t status_sequence;
  uint32_t sample_timestamp_ms;
  uint64_t session_id;
  uint32_t received_sequence;
  uint32_t applied_sequence;
  uint8_t  reject_reason;
  uint8_t  side_consistency_flags;
  uint8_t  command_ack_flags;
};

struct FirmwareHello {
  uint8_t schema;
  uint32_t capabilities;
  uint8_t commit[20];
  uint32_t hardware_revision;
  uint32_t parameter_crc32;
};

struct DiagnosticStatus {
  uint8_t schema;
  uint8_t post_done;
  uint8_t imu_status_flags;
  uint32_t post_error_flags;
  uint32_t adc_invalid_reason_flags;
  uint16_t task_timeout_mask;
  uint32_t imu_quality_flags;
  uint32_t reset_reason_flags;
  uint32_t uptime_ms;
};

// ============================================================================
// 2. CRC-8/Dallas 查表（与 STM32 robot_link_protocol 一致）
// ============================================================================

static const uint8_t CRC8_TABLE[256] PROGMEM = {
  0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
  0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
  0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
  0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
  0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
  0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
  0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
  0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
  0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
  0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
  0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
  0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
  0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
  0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
  0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
  0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
  0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
  0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
  0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
  0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
  0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
  0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
  0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
  0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
  0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
  0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
  0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
  0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
  0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
  0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
  0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
  0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
};

static uint8_t crc8(const uint8_t *data, uint16_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc = pgm_read_byte(&CRC8_TABLE[crc ^ data[i]]);
  }
  return crc;
}

static uint32_t configCrc32(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0UL);
    }
  }
  return ~crc;
}

static void readConfigRecord(uint8_t record[CONFIG_RECORD_SIZE]) {
  for (size_t i = 0; i < CONFIG_RECORD_SIZE; ++i) {
    record[i] = EEPROM.read(i);
  }
}

static bool loadConfig() {
  uint8_t record[CONFIG_RECORD_SIZE];
  readConfigRecord(record);
  if (memcmp(record, CONFIG_MAGIC, sizeof(CONFIG_MAGIC)) != 0 ||
      record[4] != CONFIG_VERSION ||
      record[5] < CONFIG_PASSWORD_MIN_LEN ||
      record[5] > CONFIG_PASSWORD_MAX_LEN) {
    memset(g_ap_password, 0, sizeof(g_ap_password));
    return false;
  }

  uint32_t stored_crc = (uint32_t)record[CONFIG_CRC_OFFSET] |
                        ((uint32_t)record[CONFIG_CRC_OFFSET + 1U] << 8) |
                        ((uint32_t)record[CONFIG_CRC_OFFSET + 2U] << 16) |
                        ((uint32_t)record[CONFIG_CRC_OFFSET + 3U] << 24);
  if (stored_crc != configCrc32(record, CONFIG_CRC_OFFSET)) {
    memset(g_ap_password, 0, sizeof(g_ap_password));
    return false;
  }

  memset(g_ap_password, 0, sizeof(g_ap_password));
  memcpy(g_ap_password, &record[6], record[5]);
  g_ap_password[record[5]] = '\0';
  return true;
}

static bool saveConfig(const String &password) {
  size_t password_len = password.length();
  if (password_len < CONFIG_PASSWORD_MIN_LEN ||
      password_len > CONFIG_PASSWORD_MAX_LEN) {
    return false;
  }

  uint8_t record[CONFIG_RECORD_SIZE] = {};
  memcpy(record, CONFIG_MAGIC, sizeof(CONFIG_MAGIC));
  record[4] = CONFIG_VERSION;
  record[5] = (uint8_t)password_len;
  memcpy(&record[6], password.c_str(), password_len);
  uint32_t crc = configCrc32(record, CONFIG_CRC_OFFSET);
  record[CONFIG_CRC_OFFSET] = (uint8_t)(crc & 0xFFU);
  record[CONFIG_CRC_OFFSET + 1U] = (uint8_t)((crc >> 8) & 0xFFU);
  record[CONFIG_CRC_OFFSET + 2U] = (uint8_t)((crc >> 16) & 0xFFU);
  record[CONFIG_CRC_OFFSET + 3U] = (uint8_t)((crc >> 24) & 0xFFU);
  for (size_t i = 0; i < CONFIG_RECORD_SIZE; ++i) {
    EEPROM.write(i, record[i]);
  }
  if (!EEPROM.commit()) return false;
  return loadConfig();
}

// ============================================================================
// 3. robot_link_protocol 帧协议（与 STM32 一致）
// ============================================================================

#define PROTO_HEAD_0        0xA5
#define PROTO_HEAD_1        0x5A
#define PROTO_VERSION       3
#define PROTO_MAX_PAYLOAD   99
#define CMD_SET_VELOCITY    0x01
#define CMD_ESTOP           0x02
#define CMD_LINE_CTRL       0x03
#define CMD_CLEAR_FAULT     0x04
#define CMD_GET_INFO        0x05
#define CMD_HELLO           0x80
#define CMD_STATUS          0x81
#define CMD_DIAGNOSTIC      0x82
#define VELOCITY_PAYLOAD_LEN 23
#define HELLO_PAYLOAD_LEN   34
#define STATUS_PAYLOAD_LEN  92
#define DIAGNOSTIC_PAYLOAD_LEN 28
#define REQUIRED_CAPABILITIES 0x1FUL
#define ACK_APPLIED            (1U << 2)
#define ACK_REJECTED           (1U << 4)
#define COMMAND_KEEPALIVE_MS   50UL
#define COMMAND_ACK_TIMEOUT_MS 150UL

#define STATUS_FLAG_ESTOP           (1U << 0)
#define STATUS_FLAG_FAULT_STOP      (1U << 1)
#define STATUS_FLAG_LINE_ENABLED    (1U << 2)
#define STATUS_FLAG_SPEED_VALID_ALL (1U << 3)

// 组装帧：out 至少需 (payload_len + 5) 字节
static uint16_t buildFrame(uint8_t cmd, const uint8_t *payload,
                           uint8_t payload_len, uint8_t *out, uint16_t out_len) {
  uint8_t cmd_len = 1 + payload_len;
  uint16_t frame_len = (uint16_t)cmd_len + 4;
  if (out_len < frame_len || payload_len > PROTO_MAX_PAYLOAD) return 0;
  if (payload_len > 0 && !payload) return 0;

  out[0] = PROTO_HEAD_0;
  out[1] = PROTO_HEAD_1;
  out[2] = cmd_len;
  out[3] = cmd;
  if (payload_len > 0) memcpy(&out[4], payload, payload_len);
  out[frame_len - 1] = crc8(&out[2], (uint16_t)cmd_len + 1);
  return frame_len;
}

// 构造速度指令帧
static void writeU32LE(uint8_t *out, uint32_t value) {
  out[0] = (uint8_t)value;
  out[1] = (uint8_t)(value >> 8);
  out[2] = (uint8_t)(value >> 16);
  out[3] = (uint8_t)(value >> 24);
}

static void writeU64LE(uint8_t *out, uint64_t value) {
  writeU32LE(out, (uint32_t)value);
  writeU32LE(&out[4], (uint32_t)(value >> 32));
}

static uint16_t buildVelocityFrame(float linear_x, float angular_z, uint8_t enable,
                                   uint64_t session_id, uint32_t sequence,
                                   uint8_t *out, uint16_t out_len) {
  uint8_t payload[VELOCITY_PAYLOAD_LEN];
  payload[0] = PROTO_VERSION;
  memcpy(&payload[1], &linear_x, 4);
  memcpy(&payload[5], &angular_z, 4);
  payload[9] = enable;
  payload[10] = 2;  // ESP12F session producer
  writeU64LE(&payload[11], session_id);
  writeU32LE(&payload[19], sequence);
  return buildFrame(CMD_SET_VELOCITY, payload, VELOCITY_PAYLOAD_LEN, out, out_len);
}

// 构造 ESTOP 帧
static uint16_t buildEstopFrame(uint8_t stop, uint8_t *out, uint16_t out_len) {
  uint8_t payload[2] = {PROTO_VERSION, stop};
  return buildFrame(CMD_ESTOP, payload, sizeof(payload), out, out_len);
}

// 构造巡线控制帧
static uint16_t buildLineCtrlFrame(uint8_t enable, uint8_t *out, uint16_t out_len) {
  uint8_t payload[2] = {PROTO_VERSION, enable};
  return buildFrame(CMD_LINE_CTRL, payload, sizeof(payload), out, out_len);
}

static uint16_t buildClearFaultFrame(uint8_t *out, uint16_t out_len) {
  const uint8_t version = PROTO_VERSION;
  return buildFrame(CMD_CLEAR_FAULT, &version, 1, out, out_len);
}

static uint16_t buildGetInfoFrame(uint8_t *out, uint16_t out_len) {
  const uint8_t version = PROTO_VERSION;
  return buildFrame(CMD_GET_INFO, &version, 1, out, out_len);
}

// 解析 STM32 状态帧
static bool parseStatusFrame(const uint8_t *payload, uint8_t len, ChassisStatus &s) {
  if (len != STATUS_PAYLOAD_LEN) return false;
  uint8_t off = 0;

  auto readU8 = [&]() -> uint8_t { return payload[off++]; };
  auto readU16 = [&]() -> uint16_t {
    uint16_t raw = (uint16_t)(payload[off] | ((uint16_t)payload[off + 1] << 8));
    off += 2; return raw;
  };
  auto readI16 = [&]() -> int16_t { return (int16_t)readU16(); };
  auto readU32 = [&]() -> uint32_t {
    uint32_t raw = (uint32_t)payload[off] | ((uint32_t)payload[off + 1] << 8) |
                   ((uint32_t)payload[off + 2] << 16) | ((uint32_t)payload[off + 3] << 24);
    off += 4; return raw;
  };
  auto readI32 = [&]() -> int32_t { return (int32_t)readU32(); };

  s.protocol_version = readU8();
  if (s.protocol_version != PROTO_VERSION) return false;
  s.status_flags = readU8();
  s.control_source = readU8();
  s.motor_enabled_mask = readU8();
  s.error_flags = readU32();
  s.latched_error_flags = readU32();
  s.battery_mv = readU16();
  for (int i = 0; i < 4; i++) s.motor_speed_mmps[i] = readI16();
  for (int i = 0; i < 4; i++) s.encoder_count[i] = readI32();
  for (int i = 0; i < 4; i++) s.motor_current_ma[i] = readU16();
  for (int i = 0; i < 4; i++) s.motor_target_mmps[i] = readI16();
  for (int i = 0; i < 4; i++) s.motor_output_permille[i] = readI16();
  s.motor_speed_valid_mask = readU8();
  s.encoder_anomaly_mask = readU8();
  s.comm_health_flags = readU8();
  s.status_sequence = readU32();
  s.sample_timestamp_ms = readU32();
  uint32_t session_low = readU32();
  uint32_t session_high = readU32();
  s.session_id = (uint64_t)session_low | ((uint64_t)session_high << 32);
  s.received_sequence = readU32();
  s.applied_sequence = readU32();
  s.reject_reason = readU8();
  s.side_consistency_flags = readU8();
  s.command_ack_flags = readU8();
  return true;
}

static uint32_t readU32LE(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static bool parseHelloFrame(const uint8_t *payload, uint8_t len, FirmwareHello &hello) {
  if (len != HELLO_PAYLOAD_LEN || payload[0] != PROTO_VERSION) return false;
  hello.schema = payload[1];
  hello.capabilities = readU32LE(&payload[2]);
  memcpy(hello.commit, &payload[6], sizeof(hello.commit));
  hello.hardware_revision = readU32LE(&payload[26]);
  hello.parameter_crc32 = readU32LE(&payload[30]);
  return hello.schema == 1U && hello.capabilities == REQUIRED_CAPABILITIES;
}

static bool parseDiagnosticFrame(const uint8_t *payload, uint8_t len, DiagnosticStatus &d) {
  if (len != DIAGNOSTIC_PAYLOAD_LEN || payload[0] != PROTO_VERSION || payload[1] != 1U) return false;
  d.schema = payload[1];
  d.post_done = payload[2];
  d.imu_status_flags = payload[3];
  d.post_error_flags = readU32LE(&payload[4]);
  d.adc_invalid_reason_flags = readU32LE(&payload[8]);
  d.task_timeout_mask = (uint16_t)payload[12] | ((uint16_t)payload[13] << 8);
  d.imu_quality_flags = readU32LE(&payload[16]);
  d.reset_reason_flags = readU32LE(&payload[20]);
  d.uptime_ms = readU32LE(&payload[24]);
  return true;
}

// ============================================================================
// 4. 帧接收状态机
// ============================================================================

#define RX_BUF_SIZE 128
static uint8_t  rx_buf[RX_BUF_SIZE];
static uint16_t rx_pos = 0;
static esp_frame_parser_t g_rx_parser = {};

// ============================================================================
// 5. WiFi 管理（首次配置 AP / 受保护控制 AP）
// ============================================================================

static String my_ip;

static void wifiStartAP() {
  WiFi.mode(WIFI_AP);
  if (g_config_valid) {
    WiFi.softAP(AP_SSID, g_ap_password);
  } else {
    WiFi.softAP(SETUP_AP_SSID);
  }
  my_ip = WiFi.softAPIP().toString();
}

static void wifiInit() {
  WiFi.persistent(false);
  wifiStartAP();
}

static float jsonFloatValue(const String &cmd, const char *key, float fallback) {
  int idx = cmd.indexOf(key);
  if (idx < 0) return fallback;
  idx += strlen(key);
  while (idx < (int)cmd.length() && (cmd[idx] == ' ' || cmd[idx] == '"')) idx++;
  return cmd.substring(idx).toFloat();
}

// ============================================================================
// 6. WebSocket + HTTP 服务器
// ============================================================================

static ESP8266WebServer    http(HTTP_PORT);
static WebSocketsServer    ws(WEB_SOCKET_PORT);
static ChassisStatus       g_status = {};
static FirmwareHello       g_hello = {};
static DiagnosticStatus    g_diagnostic = {};
static esp_link_policy_t   g_link_policy = {};
#define NO_OWNER 0xFFU
#define OWNER_LEASE_MS 500UL
#define OWNER_HEARTBEAT_MS 200UL
static uint8_t             g_owner_client = NO_OWNER;
static uint8_t             g_ws_client_count = 0;
static unsigned long       g_owner_last_heartbeat_ms = 0;
static unsigned long       g_last_telem_ms = 0;
static unsigned long       g_last_vel_ms = 0;
static bool                g_vel_active = false;
static float               g_lx = 0, g_az = 0;
static uint64_t            g_session_id = 0;
static uint32_t            g_command_sequence = 1U;
static uint32_t            g_last_sent_sequence = 0U;
static unsigned long       g_last_command_tx_ms = 0;
static unsigned long       g_command_pending_since_ms = 0;
static unsigned long       g_last_get_info_ms = 0;
static bool                g_hello_valid = false;
static bool                g_status_valid = false;
static bool                g_release_acked = false;
static bool                g_command_ack_pending = false;
static uint8_t             g_last_sent_enable = 0U;
static bool                g_safety_blocked = false;

static bool controlHandshakeReady() {
  return g_hello_valid && g_status_valid && g_release_acked && !g_safety_blocked;
}

static unsigned long statusAgeMs(unsigned long now) {
  return EspLinkPolicy_StatusAge(&g_link_policy, now);
}

// 发送帧到 STM32
static void sendToSTM32(const uint8_t *data, uint16_t len) {
  Serial.write(data, len);
}

static bool clientIsOwner(uint8_t num) {
  return g_owner_client != NO_OWNER && num == g_owner_client;
}

static void sendControlRole(uint8_t num, const char *role) {
  String response = String("{\"type\":\"control\",\"role\":\"") + role +
                    "\",\"lease_ms\":" + OWNER_LEASE_MS +
                    ",\"heartbeat_ms\":" + OWNER_HEARTBEAT_MS + "}";
  ws.sendTXT(num, response);
}

static void sendVelocityCommand(float linear_x, float angular_z, uint8_t enable,
                                bool advance_sequence) {
  if (advance_sequence) {
    g_command_sequence++;
  }
  uint8_t buf[PROTO_MAX_PAYLOAD + 5];
  uint16_t len = buildVelocityFrame(linear_x, angular_z, enable, g_session_id,
                                    g_command_sequence, buf, sizeof(buf));
  if (len) {
    sendToSTM32(buf, len);
    g_last_sent_sequence = g_command_sequence;
    g_last_command_tx_ms = millis();
    g_last_sent_enable = enable;
    if (enable == 0U && advance_sequence) g_release_acked = false;
    if (advance_sequence) {
      g_command_ack_pending = true;
      g_command_pending_since_ms = g_last_command_tx_ms;
    }
  }
}

static void stopChassis() {
  sendVelocityCommand(0.0f, 0.0f, 0U, true);
  g_vel_active = false;
  g_lx = 0.0f;
  g_az = 0.0f;
}

static void sendNeutralControl() {
  stopChassis();
  uint8_t buf[PROTO_MAX_PAYLOAD + 5];
  uint16_t len = buildLineCtrlFrame(0U, buf, sizeof(buf));
  if (len) sendToSTM32(buf, len);
}

static void releaseOwnerAndStop() {
  sendNeutralControl();
  g_owner_client = NO_OWNER;
  g_owner_last_heartbeat_ms = 0;
  ws.broadcastTXT("{\"type\":\"control\",\"role\":\"available\"}");
}

static void failClosedAndRequireHandshake() {
  sendVelocityCommand(0.0f, 0.0f, 0U, true);
  g_release_acked = false;
  g_vel_active = false;
  g_lx = 0.0f;
  g_az = 0.0f;
  if (g_owner_client != NO_OWNER) {
    g_owner_client = NO_OWNER;
    g_owner_last_heartbeat_ms = 0;
    ws.broadcastTXT("{\"type\":\"control\",\"role\":\"readonly\"}");
  }
}

// WebSocket 事件处理
static void wsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
  case WStype_CONNECTED:
    if (g_ws_client_count < 0xFFU) g_ws_client_count++;
    sendControlRole(num, controlHandshakeReady() && g_owner_client == NO_OWNER ? "available" : "readonly");
    break;

  case WStype_DISCONNECTED:
    if (g_ws_client_count > 0U) g_ws_client_count--;
    if (clientIsOwner(num)) {
      releaseOwnerAndStop();
    }
    break;

  case WStype_TEXT: {
    // 解析 JSON 指令
    String cmd;
    cmd.reserve(length + 1);
    for (size_t i = 0; i < length; ++i) {
      cmd += (char)payload[i];
    }

    if (cmd.startsWith("{\"cmd\":\"claim\"")) {
      if (!g_link_policy.online || !controlHandshakeReady()) {
        sendControlRole(num, "offline");
        break;
      }
      if (g_owner_client == NO_OWNER || clientIsOwner(num)) {
        g_owner_client = num;
        g_owner_last_heartbeat_ms = millis();
        sendControlRole(num, "owner");
      } else {
        sendControlRole(num, "readonly");
      }
    }
    else if (cmd.startsWith("{\"cmd\":\"heartbeat\"")) {
      if (clientIsOwner(num)) {
        g_owner_last_heartbeat_ms = millis();
        sendControlRole(num, "owner");
      } else {
        sendControlRole(num, "readonly");
      }
    }
    else if (cmd.startsWith("{\"cmd\":\"estop\"")) {
      uint8_t buf[PROTO_MAX_PAYLOAD + 5];
      uint16_t len = buildEstopFrame(1U, buf, sizeof(buf));
      if (len) sendToSTM32(buf, len);
      g_vel_active = false;
    }
    else if (cmd.startsWith("{\"cmd\":\"clearfault\"")) {
      if (!clientIsOwner(num)) {
        sendControlRole(num, "readonly");
        break;
      }
      if ((g_status.status_flags & STATUS_FLAG_ESTOP) != 0U) {
        ws.sendTXT(num, "{\"type\":\"faultclear\",\"result\":\"estop_local_only\"}");
        break;
      }
      uint8_t buf[PROTO_MAX_PAYLOAD + 5];
      uint16_t len = buildClearFaultFrame(buf, sizeof(buf));
      if (len) sendToSTM32(buf, len);
      ws.sendTXT(num, "{\"type\":\"faultclear\",\"result\":\"queued\"}");
    }
    else if (cmd.startsWith("{\"cmd\":\"vel\"")) {
      if (!clientIsOwner(num)) {
        sendControlRole(num, "readonly");
        break;
      }
      if (!g_link_policy.online) {
        sendControlRole(num, "offline");
        break;
      }
      // {"cmd":"vel","lx":0.30,"az":0.00}
      int lxIdx = cmd.indexOf("\"lx\":");
      int azIdx = cmd.indexOf("\"az\":");
      if (lxIdx > 0 && azIdx > 0) {
        float next_lx = jsonFloatValue(cmd, "\"lx\":", g_lx);
        float next_az = jsonFloatValue(cmd, "\"az\":", g_az);
        // 钳位
        if (next_lx >  MAX_LINEAR_MPS)  next_lx =  MAX_LINEAR_MPS;
        if (next_lx < -MAX_LINEAR_MPS)  next_lx = -MAX_LINEAR_MPS;
        if (next_az >  MAX_ANGULAR_RPS) next_az =  MAX_ANGULAR_RPS;
        if (next_az < -MAX_ANGULAR_RPS) next_az = -MAX_ANGULAR_RPS;
        bool changed = !g_vel_active || next_lx != g_lx || next_az != g_az;
        g_lx = next_lx;
        g_az = next_az;
        g_vel_active = true;
        g_last_vel_ms = millis();

        sendVelocityCommand(g_lx, g_az, 1U, changed);
      }
    }
    else if (cmd.startsWith("{\"cmd\":\"stop\"")) {
      if (!clientIsOwner(num)) {
        sendControlRole(num, "readonly");
        break;
      }
      stopChassis();
    }
    else if (cmd.startsWith("{\"cmd\":\"line\"")) {
      if (!clientIsOwner(num)) {
        sendControlRole(num, "readonly");
        break;
      }
      if (!g_link_policy.online) {
        sendControlRole(num, "offline");
        break;
      }
      // {"cmd":"line","v":1}  or  {"cmd":"line","v":0}
      int vIdx = cmd.indexOf("\"v\":");
      uint8_t v = (vIdx > 0) ? (uint8_t)cmd.substring(vIdx + 4).toInt() : 0;
      uint8_t buf[PROTO_MAX_PAYLOAD + 5];
      uint16_t len = buildLineCtrlFrame(v, buf, sizeof(buf));
      if (len) sendToSTM32(buf, len);
    }
    break;
  }
  default: break;
  }
}

// ============================================================================
// 7. Web 控制页面（PROGMEM，需在 handleRoot 之前定义）
// ============================================================================

#if 0  // 页面源码保留用于生成 html_page_gz.h，运行时不再占用 Flash 两份空间
static const char HTML_PAGE[] PROGMEM = R"raw(
<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no,viewport-fit=cover,maximum-scale=1">
<title>F407 底盘控制</title>
<style>
:root{--bg:#F6F8FB;--surface:#FFFFFF;--elev:#FFFFFF;--primary:#2563EB;--primary-soft:#DBEAFE;--accent:#0F766E;--ok:#059669;--warn:#D97706;--danger:#DC2626;--text:#111827;--muted:#4B5563;--border:#E5E7EB;--track:#EEF2F7;--shadow:0 10px 24px rgba(15,23,42,.08)}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;min-height:100vh;user-select:none;-webkit-user-select:none}button,input{font:inherit}.app{min-height:100vh;display:flex;flex-direction:column;padding:10px 12px calc(12px + env(safe-area-inset-bottom));max-width:640px;margin:0 auto}.top{min-height:44px;display:grid;grid-template-columns:1fr auto 1fr;align-items:center;background:rgba(255,255,255,.88);border:1px solid var(--border);border-radius:8px;padding:6px 8px;gap:8px;box-shadow:var(--shadow);font-size:12px}.conn{display:flex;align-items:center;gap:6px;min-width:0;font-weight:700}.dot{width:8px;height:8px;border-radius:50%;background:var(--danger)}.connected .dot{background:var(--ok)}.ap{color:var(--muted);overflow:hidden;text-overflow:ellipsis;white-space:nowrap}.right{display:flex;align-items:center;justify-content:flex-end;gap:8px}.mono{font-family:"SF Mono","Cascadia Code",Consolas,monospace}main{display:grid;gap:10px;padding-top:10px}.dash{display:grid;grid-template-columns:1fr 1fr;gap:8px;transition:opacity .2s}.stale .dash{opacity:.55}.panel{background:var(--surface);border:1px solid var(--border);border-radius:8px;padding:10px 12px;box-shadow:var(--shadow)}.battery{display:grid;place-items:center;position:relative;min-height:92px}.battery svg{width:126px;height:72px}.battery .read{position:absolute;top:34px;text-align:center}.big{font-size:18px;font-weight:800}.small{font-size:11px;color:var(--muted)}.mode{display:grid;align-content:center;gap:8px}.mode-pill{display:inline-flex;align-items:center;justify-content:center;min-height:30px;border-radius:8px;background:var(--primary-soft);color:var(--primary);font-weight:800}.motors{grid-column:1/-1;display:grid;grid-template-columns:1fr 1fr;gap:8px}.motor{background:var(--surface);border:1px solid var(--border);border-left:3px solid var(--primary);border-radius:8px;padding:9px 10px}.motor:nth-child(n+3){border-left-color:var(--accent)}.motor.fault{border-left-color:var(--danger);background:#FEF2F2;animation:pulse .5s infinite alternate}.mh{display:flex;justify-content:space-between;font-size:11px;font-weight:800;color:var(--muted)}.kv{display:grid;grid-template-columns:42px 1fr 22px;gap:6px;margin-top:5px;align-items:baseline}.kv label{font-size:10px;color:var(--muted)}.kv span{font-size:15px;font-weight:800}.kv b{font-size:10px;color:var(--muted)}.flash{color:var(--primary);transition:color .25s}.error{grid-column:1/-1;display:none;border-left:3px solid var(--danger);background:#FEF2F2;color:var(--danger);border-radius:8px;padding:8px 10px;font-size:12px;font-weight:700}.error.show{display:block}.control{display:grid;gap:10px}.joy{height:210px;max-width:380px;width:100%;margin:0 auto;position:relative;background:linear-gradient(180deg,#FFFFFF,#F8FAFC);border:1px solid var(--border);border-radius:8px;overflow:hidden;touch-action:none;box-shadow:var(--shadow)}.joy:before,.joy:after{content:"";position:absolute;background:var(--border)}.joy:before{left:50%;top:0;width:1px;height:100%}.joy:after{left:0;top:50%;width:100%;height:1px}.joy.locked:after{content:"急停已锁定";display:grid;place-items:center;background:rgba(255,255,255,.86);inset:0;width:auto;height:auto;color:var(--danger);font-weight:900;z-index:3}.thumb{width:52px;height:52px;border-radius:50%;position:absolute;left:calc(50% - 26px);top:calc(50% - 26px);background:var(--primary);box-shadow:0 10px 22px rgba(37,99,235,.28),inset 0 0 0 7px rgba(255,255,255,.28);z-index:2;transition:left .2s ease-out,top .2s ease-out}.dragging .thumb{transition:none}.slider{display:grid;grid-template-columns:auto 1fr 48px;align-items:center;gap:8px;color:var(--muted);font-size:12px;font-weight:700}.slider input{width:100%;accent-color:var(--primary)}.buttons{display:grid;grid-template-columns:1fr 1fr;gap:8px;border-top:1px solid var(--border);padding-top:10px;margin-top:10px}.btn{height:42px;border:0;border-radius:8px;color:white;font-weight:800}.stop{grid-column:1/-1;height:48px;background:var(--danger)}.estop{background:var(--warn)}.estop.active{background:var(--danger);animation:pulse .7s infinite alternate}.line{background:#E5E7EB;color:#374151}.line.active{background:var(--ok);color:white}.btn:active{transform:scale(.98)}.btn:disabled{opacity:.45}@keyframes pulse{from{box-shadow:0 0 0 rgba(220,38,38,0)}to{box-shadow:0 0 0 4px rgba(220,38,38,.18)}}@media (min-width:700px){.app{max-width:760px}.dash{grid-template-columns:180px 1fr}.motors{grid-column:auto;grid-template-columns:1fr 1fr}.error{grid-column:1/-1}.control{grid-template-columns:1fr 260px;align-items:start}.joy{max-width:none}}
.joy.locked:after{content:attr(data-lock-reason)}
</style>
</head>
<body>
<div class="app" id="app">
  <div class="top" id="top">
    <div class="conn"><span class="dot"></span><span id="connText">重连中...</span></div>
    <div class="ap" id="apName">F407_Chassis</div>
    <div class="right"><span class="mono" id="batTop">--.-V</span></div>
  </div>
  <main>
    <section class="dash">
      <div class="panel battery">
        <svg viewBox="0 0 120 70"><path d="M12 58 A48 48 0 0 1 108 58" pathLength="100" fill="none" stroke="var(--border)" stroke-width="10" stroke-linecap="round"/><path id="batArc" d="M12 58 A48 48 0 0 1 108 58" pathLength="100" fill="none" stroke="var(--ok)" stroke-width="10" stroke-linecap="round" stroke-dasharray="100" stroke-dashoffset="100"/></svg>
        <div class="read"><div class="big" id="batMain">--.-V</div><div class="small" id="batPct">--%</div></div>
      </div>
      <div class="panel mode"><div class="small">控制来源</div><div class="mode-pill" id="mode">无</div><div class="small">协议 v<span id="proto">-</span> · <span id="fresh">超时</span></div></div>
      <div class="motors">
        <div class="motor" id="m0"><div class="mh"><span>电机 1</span><span id="v0">--</span></div><div class="kv"><label>速度</label><span class="mono" id="s0">0.00</span><b>m/s</b></div><div class="kv"><label>累计</label><span class="mono" id="e0">0</span><b></b></div><div class="kv"><label>电流</label><span class="mono" id="c0">0.00</span><b>A</b></div><div class="kv"><label>输出</label><span class="mono" id="p0">0</span><b></b></div></div>
        <div class="motor" id="m1"><div class="mh"><span>电机 2</span><span id="v1">--</span></div><div class="kv"><label>速度</label><span class="mono" id="s1">0.00</span><b>m/s</b></div><div class="kv"><label>累计</label><span class="mono" id="e1">0</span><b></b></div><div class="kv"><label>电流</label><span class="mono" id="c1">0.00</span><b>A</b></div><div class="kv"><label>输出</label><span class="mono" id="p1">0</span><b></b></div></div>
        <div class="motor" id="m2"><div class="mh"><span>电机 3</span><span id="v2">--</span></div><div class="kv"><label>速度</label><span class="mono" id="s2">0.00</span><b>m/s</b></div><div class="kv"><label>累计</label><span class="mono" id="e2">0</span><b></b></div><div class="kv"><label>电流</label><span class="mono" id="c2">0.00</span><b>A</b></div><div class="kv"><label>输出</label><span class="mono" id="p2">0</span><b></b></div></div>
        <div class="motor" id="m3"><div class="mh"><span>电机 4</span><span id="v3">--</span></div><div class="kv"><label>速度</label><span class="mono" id="s3">0.00</span><b>m/s</b></div><div class="kv"><label>累计</label><span class="mono" id="e3">0</span><b></b></div><div class="kv"><label>电流</label><span class="mono" id="c3">0.00</span><b>A</b></div><div class="kv"><label>输出</label><span class="mono" id="p3">0</span><b></b></div></div>
      </div>
      <div class="error" id="errBar">错误 0x00000000</div>
    </section>
    <section class="control">
      <div class="joy" id="joy"><div class="thumb" id="thumb"></div></div>
      <div class="panel">
        <div class="slider"><span>速度上限</span><input type="range" id="spdScale" min="10" max="100" value="60"><span class="mono" id="spdVal">60%</span></div>
        <div class="buttons"><button class="btn stop" id="btnStop">停止</button><button class="btn estop" id="btnEstop">急停</button><button class="btn line" id="btnLine">巡线关</button><button class="btn line" id="btnClearFault">清除故障</button></div>
      </div>
    </section>
  </main>
</div>
<script>
var ws,owner=false,active=false,spdScale=.6,lastSend=0,lastData=0,lx=0,az=0,estop=false,lineOn=false,latestLatched=0,clearPendingLatched=0;
var srcNames=['无','上位机','手柄','ESP12F','调试','巡线'];
var faultNames=[[0,'电池低压'],[1,'电机1过流'],[2,'电机2过流'],[3,'电机3过流'],[4,'电机4过流'],[5,'急停'],[6,'安全锁停'],[7,'编码器无效'],[8,'DRV nFAULT'],[9,'TIM1 BKIN'],[17,'编码器反馈丢失'],[18,'电池严重欠压']];
var joy=document.getElementById('joy'),thumb=document.getElementById('thumb');
function el(id){return document.getElementById(id)}
function text(id,v){var n=el(id),s=String(v);if(n.textContent!==s){n.textContent=s;n.classList.add('flash');setTimeout(function(){n.classList.remove('flash')},260)}}
function send(o){if(ws&&ws.readyState===1)ws.send(JSON.stringify(o))}
function applyControlLock(){var j=el('joy');j.dataset.lockReason=estop?'急停已锁定':'只读模式';j.classList.toggle('locked',estop||!owner);el('btnStop').disabled=estop||!owner;el('btnLine').disabled=estop||!owner;el('btnClearFault').disabled=estop||!owner}
function setRole(role){owner=role==='owner';text('connText',owner?'已连接 · 控制权':'已连接 · 只读');applyControlLock();if(role==='available')setTimeout(function(){send({cmd:'claim'})},20)}
function handleMessage(d){if(d&&d.type==='control'){setRole(d.role);return}if(d&&d.type==='faultclear'){text('btnClearFault',d.result==='queued'?'请求已入队':(d.result==='estop_local_only'?'急停需本地解除':'清除故障'));return}update(d)}
function connect(){var ip=location.hostname;ws=new WebSocket('ws://'+ip+':81');ws.onopen=function(){el('top').classList.add('connected');text('connText','申请控制权');send({cmd:'claim'})};ws.onclose=function(){owner=false;active=false;applyControlLock();el('top').classList.remove('connected');text('connText','重连中...');setTimeout(connect,2000)};ws.onerror=function(){try{ws.close()}catch(e){}};ws.onmessage=function(e){try{handleMessage(JSON.parse(e.data))}catch(_){}}}
function hex(v){return('00000000'+(v>>>0).toString(16).toUpperCase()).slice(-8)}
function decodeFaults(v){var a=[];for(var i=0;i<faultNames.length;i++)if(v&(1<<faultNames[i][0]))a.push(faultNames[i][1]);return a.length?a.join('、'):'无'}
function update(d){lastData=Date.now();el('app').classList.remove('stale');var bat=d.bat||0,pct=Math.max(0,Math.min(100,Math.round((bat-10.5)/(12.6-10.5)*100)));text('proto',d.pv);text('batTop',bat.toFixed(1)+'V');text('batMain',bat.toFixed(1)+'V');text('batPct',pct+'%');el('batArc').style.strokeDashoffset=100-pct;el('batArc').style.stroke=bat<10.5?'var(--danger)':(bat<11.5?'var(--warn)':'var(--ok)');text('mode',srcNames[d.src]||String(d.src));text('fresh','实时');estop=!!(d.flags&1);lineOn=!!(d.flags&4);applyControlLock();el('btnEstop').classList.toggle('active',estop);text('btnEstop',estop?'请本地解除':'急停');el('btnLine').classList.toggle('active',lineOn);text('btnLine',lineOn?'巡线开':'巡线关');var err=d.err>>>0,lat=d.lat>>>0;latestLatched=lat;el('errBar').classList.toggle('show',err!==0||lat!==0);text('errBar','当前：'+decodeFaults(err)+'；锁存：'+decodeFaults(lat)+'（0x'+hex(lat)+'）');if(clearPendingLatched){text('btnClearFault',lat===0?'清除成功':(lat===clearPendingLatched?'主控拒绝：条件仍存在':'部分清除'));clearPendingLatched=0}for(var i=0;i<4;i++){var en=!!(d.mask&(1<<i)),valid=!!(d.valid&(1<<i)),fault=!!(err&(1<<(i+1)));el('m'+i).classList.toggle('fault',fault);text('v'+i,en?(valid?'正常':'超时'):'关闭');text('s'+i,(d.spd[i]||0).toFixed(2));text('e'+i,d.enc[i]||0);text('c'+i,(d.cur[i]||0).toFixed(2));text('p'+i,d.pwm[i]||0)}}
function resetJoy(){active=false;joy.classList.remove('dragging');thumb.style.left='calc(50% - 26px)';thumb.style.top='calc(50% - 26px)';lx=0;az=0;if(owner){send({cmd:'vel',lx:0,az:0});send({cmd:'stop'})}}
function moveJoy(x,y,force){if(estop||!owner)return;var r=joy.getBoundingClientRect(),maxX=r.width/2-26,maxY=r.height/2-26,dx=x-r.left-r.width/2,dy=y-r.top-r.height/2;dx=Math.max(-maxX,Math.min(maxX,dx));dy=Math.max(-maxY,Math.min(maxY,dy));thumb.style.left=(r.width/2+dx-26)+'px';thumb.style.top=(r.height/2+dy-26)+'px';var dead=12;if(Math.sqrt(dx*dx+dy*dy)<dead){lx=0;az=0}else{lx=-(dy/maxY)*.5*spdScale;az=(dx/maxX)*1.0*spdScale}var now=Date.now();if(force||now-lastSend>=50){lastSend=now;send({cmd:'vel',lx:Number(lx.toFixed(3)),az:Number(az.toFixed(3))})}}
joy.addEventListener('pointerdown',function(e){if(estop||!owner)return;active=true;joy.classList.add('dragging');joy.setPointerCapture(e.pointerId);moveJoy(e.clientX,e.clientY,true)});
joy.addEventListener('pointermove',function(e){if(active)moveJoy(e.clientX,e.clientY,false)});
joy.addEventListener('pointerup',resetJoy);joy.addEventListener('pointercancel',resetJoy);
el('btnStop').onclick=function(){if(owner)resetJoy()};
el('btnEstop').onclick=function(){if(!estop)send({cmd:'estop'})};
el('btnLine').onclick=function(){if(owner&&!estop)send({cmd:'line',v:lineOn?0:1})};
el('btnClearFault').onclick=function(){if(owner&&!estop&&latestLatched){clearPendingLatched=latestLatched;send({cmd:'clearfault'})}};
el('spdScale').oninput=function(){spdScale=parseInt(this.value,10)/100;text('spdVal',this.value+'%')};
setInterval(function(){if(Date.now()-lastData>300){el('app').classList.add('stale');text('fresh','超时')}},150);
setInterval(function(){if(owner)send({cmd:'heartbeat'})},200);
connect();
</script>
</body>
</html>
)raw";
#endif

// 推送遥测 JSON 到 WebSocket 客户端
static void pushTelemetry() {
  if (g_ws_client_count == 0U || !g_link_policy.status_valid) return;
  unsigned long now = millis();
  if (now - g_last_telem_ms < TELEM_INTERVAL_MS) return;
  g_last_telem_ms = now;

  unsigned long status_age_ms = statusAgeMs(now);
  char json[768];
  snprintf(json, sizeof(json),
    "{\"pv\":%u,\"flags\":%u,\"src\":%u,\"mask\":%u,\"valid\":%u,"
    "\"err\":%lu,\"lat\":%lu,\"bat\":%.2f,"
    "\"spd\":[%.2f,%.2f,%.2f,%.2f],"
    "\"enc\":[%ld,%ld,%ld,%ld],"
    "\"cur\":[%.2f,%.2f,%.2f,%.2f],"
    "\"tgt\":[%.2f,%.2f,%.2f,%.2f],"
    "\"pwm\":[%d,%d,%d,%d],"
    "\"lx\":%.2f,\"az\":%.2f,\"online\":%s,\"status_age_ms\":%lu,"
    "\"diag\":{\"schema\":%u,\"post_done\":%u,\"imu_status\":%u,\"post\":%lu,\"adc_invalid\":%lu,\"task_timeout\":%u,\"imu_quality\":%lu,\"reset\":%lu,\"uptime_ms\":%lu},"
    "\"rx\":{\"timeout\":%lu,\"crc\":%lu,\"length\":%lu,\"uart\":%lu}}",
    g_status.protocol_version, g_status.status_flags, g_status.control_source,
    g_status.motor_enabled_mask, g_status.motor_speed_valid_mask,
    (unsigned long)g_status.error_flags,
    (unsigned long)g_status.latched_error_flags,
    (float)g_status.battery_mv / 1000.0f,
    (float)g_status.motor_speed_mmps[0] / 1000.0f,
    (float)g_status.motor_speed_mmps[1] / 1000.0f,
    (float)g_status.motor_speed_mmps[2] / 1000.0f,
    (float)g_status.motor_speed_mmps[3] / 1000.0f,
    (long)g_status.encoder_count[0], (long)g_status.encoder_count[1],
    (long)g_status.encoder_count[2], (long)g_status.encoder_count[3],
    (float)g_status.motor_current_ma[0] / 1000.0f,
    (float)g_status.motor_current_ma[1] / 1000.0f,
    (float)g_status.motor_current_ma[2] / 1000.0f,
    (float)g_status.motor_current_ma[3] / 1000.0f,
    (float)g_status.motor_target_mmps[0] / 1000.0f,
    (float)g_status.motor_target_mmps[1] / 1000.0f,
    (float)g_status.motor_target_mmps[2] / 1000.0f,
    (float)g_status.motor_target_mmps[3] / 1000.0f,
    g_status.motor_output_permille[0], g_status.motor_output_permille[1],
    g_status.motor_output_permille[2], g_status.motor_output_permille[3],
    g_lx, g_az, g_link_policy.online ? "true" : "false", status_age_ms,
    g_diagnostic.schema, g_diagnostic.post_done, g_diagnostic.imu_status_flags,
    (unsigned long)g_diagnostic.post_error_flags,
    (unsigned long)g_diagnostic.adc_invalid_reason_flags, g_diagnostic.task_timeout_mask,
    (unsigned long)g_diagnostic.imu_quality_flags, (unsigned long)g_diagnostic.reset_reason_flags,
    (unsigned long)g_diagnostic.uptime_ms,
    (unsigned long)g_rx_parser.timeout_count, (unsigned long)g_rx_parser.crc_error_count,
    (unsigned long)g_rx_parser.length_error_count, (unsigned long)g_rx_parser.uart_error_count);
  ws.broadcastTXT(json);
}

static const char SETUP_PAGE[] PROGMEM = R"setup(
<!doctype html><html lang="zh"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>F407 首次配置</title><style>body{font-family:system-ui;margin:0;background:#f4f6f8;color:#172033}main{max-width:420px;margin:12vh auto;background:#fff;padding:28px;border-radius:12px;box-shadow:0 12px 40px #14213d1f}input,button{box-sizing:border-box;width:100%;height:44px;margin-top:12px;border-radius:8px;font:inherit}input{border:1px solid #ccd3dd;padding:0 12px}button{border:0;background:#1456d8;color:#fff;font-weight:700}p{color:#536174;line-height:1.5}</style></head><body><main><h1>设置控制热点密码</h1><p>请输入 8–63 字节密码。保存后设备会重启并创建 F407_Chassis 热点。</p><form method="post" action="/configure"><input name="password" type="password" minlength="8" maxlength="63" required autocomplete="new-password"><button type="submit">保存并重启</button></form></main></body></html>
)setup";

// HTTP 路由：首页
static void handleRoot() {
  http.sendHeader("Cache-Control", "no-store");
  if (!g_config_valid) {
    http.send_P(200, "text/html; charset=utf-8", SETUP_PAGE);
    return;
  }
  http.sendHeader("Content-Encoding", "gzip");
  http.send_P(200,
              "text/html; charset=utf-8",
              reinterpret_cast<PGM_P>(HTML_PAGE_GZ),
              HTML_PAGE_GZ_LEN);
}

static void handleConfigure() {
  if (g_config_valid || g_reboot_at_ms != 0UL) {
    http.send(403, "text/plain; charset=utf-8", "configuration already completed");
    return;
  }
  String password = http.arg("password");
  if (password.length() < CONFIG_PASSWORD_MIN_LEN ||
      password.length() > CONFIG_PASSWORD_MAX_LEN) {
    http.send(400, "text/plain; charset=utf-8", "password must be 8-63 bytes");
    return;
  }
  if (!saveConfig(password)) {
    http.send(500, "text/plain; charset=utf-8", "configuration write verification failed");
    return;
  }
  g_reboot_at_ms = millis() + 750UL;
  http.send(200, "text/html; charset=utf-8", "<meta charset=utf-8><p>保存成功，设备正在重启。</p>");
}

// HTTP 路由：状态 JSON（方便调试）
static void handleStatus() {
  unsigned long now = millis();
  unsigned long status_age_ms = statusAgeMs(now);
  char json[768];
  snprintf(json, sizeof(json),
    "{\"pv\":%u,\"bat\":%.2f,\"err\":%lu,\"lat\":%lu,\"src\":%u,"
    "\"wifi\":\"%s\",\"ws\":%d,\"online\":%s,\"status_age_ms\":%lu,"
    "\"diag_schema\":%u,\"post_done\":%u,\"imu_status\":%u,\"post_errors\":%lu,"
    "\"adc_invalid\":%lu,\"task_timeout\":%u,\"imu_quality\":%lu,\"reset_reason\":%lu,\"uptime_ms\":%lu,"
    "\"rx_timeout\":%lu,\"rx_crc\":%lu,\"rx_length\":%lu,\"rx_uart\":%lu,"
    "\"hello_valid\":%s,\"capabilities\":%lu,\"hardware_revision\":%lu,\"parameter_crc32\":%lu,"
    "\"handshake_ready\":%s,\"session_low\":%lu,\"session_high\":%lu,\"sequence\":%lu,"
    "\"received_sequence\":%lu,\"applied_sequence\":%lu,\"ack_flags\":%u,\"reject_reason\":%u}",
    g_status.protocol_version,
    (float)g_status.battery_mv / 1000.0f,
    (unsigned long)g_status.error_flags,
    (unsigned long)g_status.latched_error_flags,
    g_status.control_source,
    my_ip.c_str(), g_ws_client_count, g_link_policy.online ? "true" : "false", status_age_ms,
    g_diagnostic.schema, g_diagnostic.post_done, g_diagnostic.imu_status_flags,
    (unsigned long)g_diagnostic.post_error_flags,
    (unsigned long)g_diagnostic.adc_invalid_reason_flags, g_diagnostic.task_timeout_mask,
    (unsigned long)g_diagnostic.imu_quality_flags, (unsigned long)g_diagnostic.reset_reason_flags,
    (unsigned long)g_diagnostic.uptime_ms,
    (unsigned long)g_rx_parser.timeout_count, (unsigned long)g_rx_parser.crc_error_count,
    (unsigned long)g_rx_parser.length_error_count, (unsigned long)g_rx_parser.uart_error_count,
    g_hello_valid ? "true" : "false", (unsigned long)g_hello.capabilities,
    (unsigned long)g_hello.hardware_revision, (unsigned long)g_hello.parameter_crc32,
    controlHandshakeReady() ? "true" : "false",
    (unsigned long)(uint32_t)g_session_id, (unsigned long)(uint32_t)(g_session_id >> 32),
    (unsigned long)g_command_sequence, (unsigned long)g_status.received_sequence,
    (unsigned long)g_status.applied_sequence, g_status.command_ack_flags, g_status.reject_reason);
  http.send(200, "application/json", json);
}

// ============================================================================
// 8. 初始化
// ============================================================================

void setup() {
  EspFrameParser_Init(&g_rx_parser);
  Serial.begin(UART_BAUD);
  g_session_id = ((uint64_t)ESP.getChipId() << 32) | (uint64_t)(micros() ^ ESP.getCycleCount());
  if (g_session_id == 0ULL) g_session_id = 1ULL;
  uint8_t startup_frame[PROTO_MAX_PAYLOAD + 5];
  uint16_t startup_len = buildGetInfoFrame(startup_frame, sizeof(startup_frame));
  if (startup_len) sendToSTM32(startup_frame, startup_len);
  g_last_get_info_ms = millis();
  for (uint8_t attempt = 0U; attempt < 3U; ++attempt) {
    sendVelocityCommand(0.0f, 0.0f, 0U, false);
  }
  g_command_ack_pending = true;
  g_command_pending_since_ms = millis();
  // ESP12F 的 UART0 TX/RX 默认在 GPIO1/GPIO3，与 STM32 USART2 交叉连接
  // 如需调试输出，可通过 Serial1 (GPIO2) 或禁用调试

  EEPROM.begin(CONFIG_RECORD_SIZE);
  g_config_valid = loadConfig();

  // WiFi 初始化（首次开放配置 AP；配置后使用保存的 WPA2 密码）
  wifiInit();

  // HTTP 服务器
  http.on("/", handleRoot);
  http.on("/status", handleStatus);
  http.on("/configure", HTTP_POST, handleConfigure);
  http.begin();

  // 配置完成前禁止启动 WebSocket 运动控制
  if (g_config_valid) {
    ws.begin();
    ws.onEvent(wsEvent);
  }

  // 状态 LED（ESP12F 内置 LED 在 GPIO2，低电平亮）
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
}

// ============================================================================
// 9. 主循环
// ============================================================================

void loop() {
  unsigned long now = millis();

  if (g_reboot_at_ms != 0UL && (int32_t)(now - g_reboot_at_ms) >= 0) {
    ESP.restart();
  }

  // --- 收 STM32 数据 ---
  if (Serial.hasOverrun() || Serial.hasRxError()) {
    while (Serial.available()) (void)Serial.read();
    EspFrameParser_OnUartError(&g_rx_parser);
    failClosedAndRequireHandshake();
  }
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (EspFrameParser_Feed(&g_rx_parser, b, now)) {
      // 收到有效帧
      uint8_t cmd = g_rx_parser.frame[3];
      uint8_t cmd_len = g_rx_parser.frame[2];
      uint8_t *payload = &g_rx_parser.frame[4];
      uint8_t payload_len = cmd_len - 1;

      if (cmd == CMD_STATUS && parseStatusFrame(payload, payload_len, g_status)) {
        g_status_valid = true;
        EspLinkPolicy_OnStatus(&g_link_policy, now);
        bool safety_blocked = (g_status.status_flags & (STATUS_FLAG_ESTOP | STATUS_FLAG_FAULT_STOP)) != 0U;
        if (safety_blocked && !g_safety_blocked) {
          g_safety_blocked = true;
          failClosedAndRequireHandshake();
        } else if (!safety_blocked && g_safety_blocked) {
          g_safety_blocked = false;
          sendVelocityCommand(0.0f, 0.0f, 0U, true);
        }
        if (g_status.session_id == g_session_id &&
            g_status.applied_sequence == g_last_sent_sequence &&
            (g_status.command_ack_flags & ACK_APPLIED) != 0U) {
          g_command_ack_pending = false;
          if (g_last_sent_enable == 0U) g_release_acked = true;
        } else if (g_status.session_id == g_session_id &&
                   g_status.received_sequence == g_last_sent_sequence &&
                   (g_status.command_ack_flags & ACK_REJECTED) != 0U) {
          if (g_safety_blocked) {
            g_command_ack_pending = false;
          } else {
            failClosedAndRequireHandshake();
          }
        }
      } else if (cmd == CMD_HELLO) {
        g_hello_valid = parseHelloFrame(payload, payload_len, g_hello);
      } else if (cmd == CMD_DIAGNOSTIC) {
        (void)parseDiagnosticFrame(payload, payload_len, g_diagnostic);
      }
    }
  }

  EspFrameParser_CheckTimeout(&g_rx_parser, now);
  if (!g_hello_valid && (unsigned long)(now - g_last_get_info_ms) >= 500UL) {
    uint8_t info_frame[PROTO_MAX_PAYLOAD + 5];
    uint16_t info_len = buildGetInfoFrame(info_frame, sizeof(info_frame));
    if (info_len) sendToSTM32(info_frame, info_len);
    g_last_get_info_ms = now;
  }
  if (EspLinkPolicy_Update(&g_link_policy, now)) {
    g_status_valid = false;
    failClosedAndRequireHandshake();
  }

  if (g_command_ack_pending &&
      (unsigned long)(now - g_command_pending_since_ms) > COMMAND_ACK_TIMEOUT_MS) {
    failClosedAndRequireHandshake();
  }

  // --- 推送遥测 ---
  if (g_config_valid) pushTelemetry();

  // owner 必须每 200ms heartbeat；500ms 租约到期立即停车并释放。
  if (g_owner_client != NO_OWNER &&
      (unsigned long)(now - g_owner_last_heartbeat_ms) > OWNER_LEASE_MS) {
    failClosedAndRequireHandshake();
  }

  // --- 速度指令超时保护 ---
  // 如果 500ms 没有收到 WebSocket 速度指令，自动清零（防失控）
  if (g_vel_active && (now - g_last_vel_ms > 500)) {
    failClosedAndRequireHandshake();
  } else if (g_vel_active &&
             (unsigned long)(now - g_last_command_tx_ms) >= COMMAND_KEEPALIVE_MS) {
    sendVelocityCommand(g_lx, g_az, 1U, false);
  }

  // --- HTTP / WebSocket 事件 ---
  http.handleClient();
  if (g_config_valid) ws.loop();

  // --- 心跳 LED（收到遥测时闪） ---
  if (g_link_policy.online) {
    digitalWrite(2, (now % 1000) < 100 ? LOW : HIGH);
  } else {
    digitalWrite(2, (now % 2000) < 200 ? LOW : HIGH);
  }
}
