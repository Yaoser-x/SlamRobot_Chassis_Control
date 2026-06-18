/*
 * F407_ESP12F.ino — ESP8266 WiFi 桥接固件（Arduino IDE）
 *
 * 功能：
 *   - AP+STA 智能切换：优先连接已知路由器，失败自动开热点
 *   - WebSocket 实时遥控 + 遥测回传
 *   - 手机网页操控界面（虚拟摇杆 + 速度/方向控制）
 *   - upper_protocol 帧协议与 STM32F407 USART2 通信
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
#include <WebSocketsServer.h>
#include <EEPROM.h>

// ============================================================================
// 1. 配置常量
// ============================================================================

#define UART_BAUD         115200
#define STA_TIMEOUT_MS    12000   // STA 连接超时（12s）
#define STA_RECONNECT_INTERVAL_MS 10000
#define AP_SSID           "F407_Chassis"
#define AP_PASS           "12345678"
#define WEB_SOCKET_PORT   81
#define HTTP_PORT         80
#define EEPROM_SIZE       128
#define EEPROM_MAGIC      0xA5
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_SSID_ADDR  4
#define EEPROM_PASS_ADDR  36      // EEPROM_SSID_ADDR + MAX_SSID_LEN
#define MAX_SSID_LEN      32
#define MAX_PASS_LEN      64

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
};

// ============================================================================
// 2. CRC-8/Dallas 查表（与 STM32 upper_protocol 一致）
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

// ============================================================================
// 3. upper_protocol 帧协议（与 STM32 一致）
// ============================================================================

#define PROTO_HEAD_0        0xA5
#define PROTO_HEAD_1        0x5A
#define PROTO_VERSION       2
#define PROTO_MAX_PAYLOAD   64
#define CMD_SET_VELOCITY    0x01
#define CMD_ESTOP           0x02
#define CMD_LINE_CTRL       0x03
#define CMD_STATUS          0x81
#define VELOCITY_PAYLOAD_LEN 10
#define STATUS_PAYLOAD_LEN  64

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
static uint16_t buildVelocityFrame(float linear_x, float angular_z,
                                   uint8_t enable, uint8_t *out, uint16_t out_len) {
  uint8_t payload[VELOCITY_PAYLOAD_LEN];
  memcpy(&payload[0], &linear_x, 4);
  memcpy(&payload[4], &angular_z, 4);
  payload[8] = enable;
  payload[9] = 0;  // mode: unused
  return buildFrame(CMD_SET_VELOCITY, payload, VELOCITY_PAYLOAD_LEN, out, out_len);
}

// 构造 ESTOP 帧
static uint16_t buildEstopFrame(uint8_t stop, uint8_t *out, uint16_t out_len) {
  return buildFrame(CMD_ESTOP, &stop, 1, out, out_len);
}

// 构造巡线控制帧
static uint16_t buildLineCtrlFrame(uint8_t enable, uint8_t *out, uint16_t out_len) {
  return buildFrame(CMD_LINE_CTRL, &enable, 1, out, out_len);
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
  (void)readU8();  // reserved
  return true;
}

// ============================================================================
// 4. 帧接收状态机
// ============================================================================

#define RX_BUF_SIZE 128
static uint8_t  rx_buf[RX_BUF_SIZE];
static uint16_t rx_pos = 0;
static uint8_t  rx_frame_buf[PROTO_MAX_PAYLOAD + 5];
static uint16_t rx_frame_len = 0;

enum RxState {
  WAIT_HEAD0,
  WAIT_HEAD1,
  WAIT_LEN,
  WAIT_BODY
};
static RxState rx_state = WAIT_HEAD0;
static uint8_t  rx_cmd_len = 0;
static uint16_t rx_body_idx = 0;

// 处理一字节，返回 true 表示收到一个有效帧
static bool feedRxByte(uint8_t b) {
  switch (rx_state) {
  case WAIT_HEAD0:
    if (b == PROTO_HEAD_0) rx_state = WAIT_HEAD1;
    break;
  case WAIT_HEAD1:
    if (b == PROTO_HEAD_1) {
      rx_state = WAIT_LEN;
      rx_frame_buf[0] = PROTO_HEAD_0;
      rx_frame_buf[1] = PROTO_HEAD_1;
    } else {
      rx_state = WAIT_HEAD0;
    }
    break;
  case WAIT_LEN:
    if (b >= 1 && b <= (PROTO_MAX_PAYLOAD + 1)) {
      rx_cmd_len = b;
      rx_body_idx = 0;
      rx_frame_buf[2] = b;
      rx_state = WAIT_BODY;
    } else {
      rx_state = WAIT_HEAD0;
    }
    break;
  case WAIT_BODY:
    rx_frame_buf[3 + rx_body_idx] = b;
    rx_body_idx++;
    if (rx_body_idx >= (uint16_t)rx_cmd_len + 1) {
      // 收完一帧，验证校验和
      rx_state = WAIT_HEAD0;
      uint8_t calc = crc8(&rx_frame_buf[2], (uint16_t)rx_cmd_len + 1);
      if (calc == b) {
        return true;
      }
    }
    break;
  }
  return false;
}

// ============================================================================
// 5. WiFi 管理（AP+STA 智能切换）
// ============================================================================

static char sta_ssid[MAX_SSID_LEN + 1] = "";
static char sta_pass[MAX_PASS_LEN + 1] = "";
static bool sta_configured = false;
static bool wifi_connected = false;
static String my_ip;
static unsigned long g_last_sta_retry_ms = 0;

static void eepromLoad() {
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(EEPROM_MAGIC_ADDR) == EEPROM_MAGIC) {
    for (int i = 0; i < MAX_SSID_LEN; i++) sta_ssid[i] = EEPROM.read(EEPROM_SSID_ADDR + i);
    for (int i = 0; i < MAX_PASS_LEN; i++) sta_pass[i] = EEPROM.read(EEPROM_PASS_ADDR + i);
    sta_ssid[MAX_SSID_LEN] = '\0';
    sta_pass[MAX_PASS_LEN] = '\0';
    if (strlen(sta_ssid) > 0) sta_configured = true;
  }
  EEPROM.end();
}

static void eepromSave(const char *ssid, const char *pass) {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  for (int i = 0; i < MAX_SSID_LEN; i++) EEPROM.write(EEPROM_SSID_ADDR + i, ssid[i] ? ssid[i] : 0);
  for (int i = 0; i < MAX_PASS_LEN; i++) EEPROM.write(EEPROM_PASS_ADDR + i, pass[i] ? pass[i] : 0);
  EEPROM.commit();
  EEPROM.end();
}

static void wifiStartAP() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  my_ip = WiFi.softAPIP().toString();
  wifi_connected = true;
}

static void wifiBeginSta() {
  if (sta_configured) {
    if (strlen(sta_pass) > 0) {
      WiFi.begin(sta_ssid, sta_pass);
    } else {
      WiFi.begin(sta_ssid);
    }
    g_last_sta_retry_ms = millis();
  }
}

static void wifiInit() {
  eepromLoad();
  WiFi.persistent(false);
  wifiStartAP();

  if (sta_configured) {
    wifiBeginSta();

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < STA_TIMEOUT_MS) {
      delay(500);
      // 喂 UART 收帧，不丢 STM32 数据
      while (Serial.available()) {
        uint8_t b = Serial.read();
        feedRxByte(b);
      }
    }

    if (WiFi.status() == WL_CONNECTED) {
      my_ip = WiFi.localIP().toString();
      wifi_connected = true;
    }
  }
}

static void wifiMaintain() {
  if (!sta_configured) return;
  if (WiFi.status() == WL_CONNECTED) {
    my_ip = WiFi.localIP().toString();
    wifi_connected = true;
    return;
  }
  wifi_connected = true;
  if (millis() - g_last_sta_retry_ms >= STA_RECONNECT_INTERVAL_MS) {
    WiFi.mode(WIFI_AP_STA);
    wifiBeginSta();
  }
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
static bool                g_status_valid = false;
static uint8_t             g_ws_client_id = 0;
static bool                g_ws_connected = false;
static unsigned long       g_last_telem_ms = 0;
static unsigned long       g_last_vel_ms = 0;
static bool                g_vel_active = false;
static float               g_lx = 0, g_az = 0;

// 发送帧到 STM32
static void sendToSTM32(const uint8_t *data, uint16_t len) {
  Serial.write(data, len);
}

// WebSocket 事件处理
static void wsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
  case WStype_CONNECTED:
    g_ws_client_id = num;
    g_ws_connected = true;
    break;

  case WStype_DISCONNECTED:
    if (num == g_ws_client_id) {
      g_ws_connected = false;
      g_ws_client_id = 0;
      // 客户端断开 → 停止底盘
      uint8_t buf[PROTO_MAX_PAYLOAD + 5];
      uint16_t len = buildVelocityFrame(0, 0, 0, buf, sizeof(buf));
      if (len) sendToSTM32(buf, len);
      g_vel_active = false;
    }
    break;

  case WStype_TEXT: {
    // 解析 JSON 指令
    String cmd;
    cmd.reserve(length + 1);
    for (size_t i = 0; i < length; ++i) {
      cmd += (char)payload[i];
    }

    if (cmd.startsWith("{\"cmd\":\"vel\"")) {
      // {"cmd":"vel","lx":0.30,"az":0.00}
      int lxIdx = cmd.indexOf("\"lx\":");
      int azIdx = cmd.indexOf("\"az\":");
      if (lxIdx > 0 && azIdx > 0) {
        g_lx = jsonFloatValue(cmd, "\"lx\":", g_lx);
        g_az = jsonFloatValue(cmd, "\"az\":", g_az);
        // 钳位
        if (g_lx >  MAX_LINEAR_MPS)  g_lx =  MAX_LINEAR_MPS;
        if (g_lx < -MAX_LINEAR_MPS)  g_lx = -MAX_LINEAR_MPS;
        if (g_az >  MAX_ANGULAR_RPS) g_az =  MAX_ANGULAR_RPS;
        if (g_az < -MAX_ANGULAR_RPS) g_az = -MAX_ANGULAR_RPS;
        g_vel_active = true;
        g_last_vel_ms = millis();

        uint8_t buf[PROTO_MAX_PAYLOAD + 5];
        uint16_t len = buildVelocityFrame(g_lx, g_az, 1, buf, sizeof(buf));
        if (len) sendToSTM32(buf, len);
      }
    }
    else if (cmd.startsWith("{\"cmd\":\"stop\"")) {
      g_vel_active = false;
      uint8_t buf[PROTO_MAX_PAYLOAD + 5];
      uint16_t len = buildVelocityFrame(0, 0, 0, buf, sizeof(buf));
      if (len) sendToSTM32(buf, len);
    }
    else if (cmd.startsWith("{\"cmd\":\"estop\"")) {
      // {"cmd":"estop","v":1}  or  {"cmd":"estop","v":0}
      int vIdx = cmd.indexOf("\"v\":");
      uint8_t v = (vIdx > 0) ? (uint8_t)cmd.substring(vIdx + 4).toInt() : 1;
      uint8_t buf[PROTO_MAX_PAYLOAD + 5];
      uint16_t len = buildEstopFrame(v, buf, sizeof(buf));
      if (len) sendToSTM32(buf, len);
      if (v) g_vel_active = false;
    }
    else if (cmd.startsWith("{\"cmd\":\"line\"")) {
      // {"cmd":"line","v":1}  or  {"cmd":"line","v":0}
      int vIdx = cmd.indexOf("\"v\":");
      uint8_t v = (vIdx > 0) ? (uint8_t)cmd.substring(vIdx + 4).toInt() : 0;
      uint8_t buf[PROTO_MAX_PAYLOAD + 5];
      uint16_t len = buildLineCtrlFrame(v, buf, sizeof(buf));
      if (len) sendToSTM32(buf, len);
    }
    else if (cmd.startsWith("{\"cmd\":\"config\"")) {
      // {"cmd":"config","ssid":"xxx","pass":"yyy"}
      int ssidIdx = cmd.indexOf("\"ssid\":\"");
      int passIdx = cmd.indexOf("\"pass\":\"");
      if (ssidIdx > 0 && passIdx > 0) {
        String ssid = cmd.substring(ssidIdx + 8);
        ssid = ssid.substring(0, ssid.indexOf('"'));
        String pass = cmd.substring(passIdx + 8);
        pass = pass.substring(0, pass.indexOf('"'));
        ssid.toCharArray(sta_ssid, MAX_SSID_LEN + 1);
        pass.toCharArray(sta_pass, MAX_PASS_LEN + 1);
        eepromSave(sta_ssid, sta_pass);
        ws.sendTXT(num, "{\"msg\":\"WiFi 配置已保存，重启后生效。\"}");
      }
    }
    break;
  }
  default: break;
  }
}

// ============================================================================
// 7. Web 控制页面（PROGMEM，需在 handleRoot 之前定义）
// ============================================================================

static const char HTML_PAGE[] PROGMEM = R"raw(
<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no,viewport-fit=cover,maximum-scale=1">
<title>F407 底盘控制</title>
<style>
:root{--bg:#F6F8FB;--surface:#FFFFFF;--elev:#FFFFFF;--primary:#2563EB;--primary-soft:#DBEAFE;--accent:#0F766E;--ok:#059669;--warn:#D97706;--danger:#DC2626;--text:#111827;--muted:#6B7280;--border:#E5E7EB;--track:#EEF2F7;--shadow:0 10px 24px rgba(15,23,42,.08)}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;min-height:100vh;user-select:none;-webkit-user-select:none}button,input{font:inherit}.app{min-height:100vh;display:flex;flex-direction:column;padding:10px 12px calc(12px + env(safe-area-inset-bottom));max-width:640px;margin:0 auto}.top{min-height:44px;display:grid;grid-template-columns:1fr auto 1fr;align-items:center;background:rgba(255,255,255,.88);border:1px solid var(--border);border-radius:8px;padding:6px 8px;gap:8px;box-shadow:var(--shadow);font-size:12px}.conn{display:flex;align-items:center;gap:6px;min-width:0;font-weight:700}.dot{width:8px;height:8px;border-radius:50%;background:var(--danger)}.connected .dot{background:var(--ok)}.ap{color:var(--muted);overflow:hidden;text-overflow:ellipsis;white-space:nowrap}.right{display:flex;align-items:center;justify-content:flex-end;gap:8px}.mono{font-family:"SF Mono","Cascadia Code",Consolas,monospace}.icon{width:32px;height:32px;border:1px solid transparent;background:transparent;color:var(--muted);display:grid;place-items:center;border-radius:8px}.icon:active{background:var(--track);border-color:var(--border)}main{display:grid;gap:10px;padding-top:10px}.dash{display:grid;grid-template-columns:1fr 1fr;gap:8px;transition:opacity .2s}.stale .dash{opacity:.55}.panel{background:var(--surface);border:1px solid var(--border);border-radius:8px;padding:10px 12px;box-shadow:var(--shadow)}.battery{display:grid;place-items:center;position:relative;min-height:92px}.battery svg{width:126px;height:72px}.battery .read{position:absolute;top:34px;text-align:center}.big{font-size:18px;font-weight:800}.small{font-size:11px;color:var(--muted)}.mode{display:grid;align-content:center;gap:8px}.mode-pill{display:inline-flex;align-items:center;justify-content:center;min-height:30px;border-radius:8px;background:var(--primary-soft);color:var(--primary);font-weight:800}.motors{grid-column:1/-1;display:grid;grid-template-columns:1fr 1fr;gap:8px}.motor{background:var(--surface);border:1px solid var(--border);border-left:3px solid var(--primary);border-radius:8px;padding:9px 10px}.motor:nth-child(n+3){border-left-color:var(--accent)}.motor.fault{border-left-color:var(--danger);background:#FEF2F2;animation:pulse .5s infinite alternate}.mh{display:flex;justify-content:space-between;font-size:11px;font-weight:800;color:var(--muted)}.kv{display:grid;grid-template-columns:42px 1fr 22px;gap:6px;margin-top:5px;align-items:baseline}.kv label{font-size:10px;color:var(--muted)}.kv span{font-size:15px;font-weight:800}.kv b{font-size:10px;color:var(--muted)}.flash{color:var(--primary);transition:color .25s}.error{grid-column:1/-1;display:none;border-left:3px solid var(--danger);background:#FEF2F2;color:var(--danger);border-radius:8px;padding:8px 10px;font-size:12px;font-weight:700}.error.show{display:block}.control{display:grid;gap:10px}.joy{height:210px;max-width:380px;width:100%;margin:0 auto;position:relative;background:linear-gradient(180deg,#FFFFFF,#F8FAFC);border:1px solid var(--border);border-radius:8px;overflow:hidden;touch-action:none;box-shadow:var(--shadow)}.joy:before,.joy:after{content:"";position:absolute;background:var(--border)}.joy:before{left:50%;top:0;width:1px;height:100%}.joy:after{left:0;top:50%;width:100%;height:1px}.joy.locked:after{content:"急停已锁定";display:grid;place-items:center;background:rgba(255,255,255,.86);inset:0;width:auto;height:auto;color:var(--danger);font-weight:900;z-index:3}.thumb{width:52px;height:52px;border-radius:50%;position:absolute;left:calc(50% - 26px);top:calc(50% - 26px);background:var(--primary);box-shadow:0 10px 22px rgba(37,99,235,.28),inset 0 0 0 7px rgba(255,255,255,.28);z-index:2;transition:left .2s ease-out,top .2s ease-out}.dragging .thumb{transition:none}.slider{display:grid;grid-template-columns:auto 1fr 48px;align-items:center;gap:8px;color:var(--muted);font-size:12px;font-weight:700}.slider input{width:100%;accent-color:var(--primary)}.buttons{display:grid;grid-template-columns:1fr 1fr;gap:8px}.btn{height:42px;border:0;border-radius:8px;color:white;font-weight:800}.stop{grid-column:1/-1;height:48px;background:var(--danger)}.estop{background:var(--warn)}.estop.active{background:var(--danger);animation:pulse .7s infinite alternate}.line{background:#E5E7EB;color:#374151}.line.active{background:var(--ok);color:white}.btn:active{transform:scale(.98)}.btn:disabled{opacity:.45}.drawer{position:fixed;inset:0;pointer-events:none;z-index:10}.drawer.open{pointer-events:auto}.shade{position:absolute;inset:0;background:rgba(17,24,39,.35);opacity:0;transition:opacity .25s}.drawer.open .shade{opacity:1}.sheet{position:absolute;right:0;top:0;height:100%;width:min(320px,88vw);background:var(--elev);border-left:1px solid var(--border);padding:14px;transform:translateX(100%);transition:transform .25s ease-out;display:grid;align-content:start;gap:12px;box-shadow:-12px 0 32px rgba(15,23,42,.12)}.drawer.open .sheet{transform:translateX(0)}.sheet-head{display:flex;align-items:center;justify-content:space-between}.field{display:grid;gap:5px}.field label{font-size:11px;color:var(--muted);font-weight:700}.field input{height:42px;border-radius:8px;border:1px solid var(--border);background:#F9FAFB;color:var(--text);padding:0 10px}.save{height:42px;border:0;border-radius:8px;background:var(--primary);color:white;font-weight:800}.save:disabled{opacity:.4}.info{border:1px solid var(--border);border-radius:8px;padding:10px;color:var(--muted);font-size:12px;background:#F9FAFB}.toast{position:fixed;left:50%;bottom:18px;transform:translateX(-50%);background:var(--text);border-radius:8px;padding:8px 12px;color:white;display:none;z-index:12;box-shadow:var(--shadow)}.toast.show{display:block}@keyframes pulse{from{box-shadow:0 0 0 rgba(220,38,38,0)}to{box-shadow:0 0 0 4px rgba(220,38,38,.18)}}@media (min-width:700px){.app{max-width:760px}.dash{grid-template-columns:180px 1fr}.motors{grid-column:auto;grid-template-columns:1fr 1fr}.error{grid-column:1/-1}.control{grid-template-columns:1fr 260px;align-items:start}.joy{max-width:none}}
</style>
</head>
<body>
<div class="app" id="app">
  <div class="top" id="top">
    <div class="conn"><span class="dot"></span><span id="connText">重连中...</span></div>
    <div class="ap" id="apName">F407_Chassis</div>
    <div class="right"><span class="mono" id="batTop">--.-V</span><button class="icon" id="openSettings" aria-label="网络设置"><svg viewBox="0 0 24 24" width="22" height="22" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 15.5A3.5 3.5 0 1 0 12 8a3.5 3.5 0 0 0 0 7.5Z"/><path d="M19.4 15a1.7 1.7 0 0 0 .3 1.9l.1.1-2 3.4-.2-.1a1.8 1.8 0 0 0-2.1.2 1.8 1.8 0 0 0-.8 1.8V22H9.3v-.3a1.8 1.8 0 0 0-.8-1.8 1.8 1.8 0 0 0-2.1-.2l-.2.1-2-3.4.1-.1A1.7 1.7 0 0 0 4.6 15a1.7 1.7 0 0 0-1.4-1.2H3V10h.2a1.7 1.7 0 0 0 1.4-1.2 1.7 1.7 0 0 0-.3-1.9l-.1-.1 2-3.4.2.1a1.8 1.8 0 0 0 2.1-.2 1.8 1.8 0 0 0 .8-1.8V1h5.4v.3a1.8 1.8 0 0 0 .8 1.8 1.8 1.8 0 0 0 2.1.2l.2-.1 2 3.4-.1.1a1.7 1.7 0 0 0-.3 1.9A1.7 1.7 0 0 0 20.8 10h.2v3.8h-.2A1.7 1.7 0 0 0 19.4 15Z"/></svg></button></div>
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
        <div class="buttons" style="margin-top:10px"><button class="btn stop" id="btnStop">停止</button><button class="btn estop" id="btnEstop">急停</button><button class="btn line" id="btnLine">巡线关</button></div>
      </div>
    </section>
  </main>
</div>
<div class="drawer" id="drawer"><div class="shade" id="shade"></div><div class="sheet"><div class="sheet-head"><strong>网络设置</strong><button class="icon" id="closeSettings" aria-label="关闭"><svg viewBox="0 0 24 24" width="22" height="22" fill="none" stroke="currentColor" stroke-width="2"><path d="M18 6 6 18M6 6l12 12"/></svg></button></div><div class="field"><label>WiFi 名称</label><input id="ssid" maxlength="32" autocomplete="off"></div><div class="field"><label>WiFi 密码</label><input id="pass" maxlength="64" type="password"></div><button class="save" id="saveWifi" disabled>保存配置</button><div class="info">IP: <span id="ipInfo">-</span><br>WebSocket: <span id="wsInfo">离线</span></div></div></div>
<div class="toast" id="toast"></div>
<script>
var ws,active=false,spdScale=.6,lastSend=0,lastData=0,lx=0,az=0,estop=false,lineOn=false;
var srcNames=['无','上位机','手柄','ESP12F','调试','巡线'];
var joy=document.getElementById('joy'),thumb=document.getElementById('thumb');
function el(id){return document.getElementById(id)}
function text(id,v){var n=el(id),s=String(v);if(n.textContent!==s){n.textContent=s;n.classList.add('flash');setTimeout(function(){n.classList.remove('flash')},260)}}
function send(o){if(ws&&ws.readyState===1)ws.send(JSON.stringify(o))}
function toast(msg){var t=el('toast');t.textContent=msg;t.classList.add('show');setTimeout(function(){t.classList.remove('show')},2200)}
function connect(){var ip=location.hostname;ws=new WebSocket('ws://'+ip+':81');ws.onopen=function(){el('top').classList.add('connected');text('connText','已连接');text('ipInfo',ip);text('wsInfo','在线')};ws.onclose=function(){el('top').classList.remove('connected');el('drawer').classList.remove('open');text('connText','重连中...');text('wsInfo','离线');setTimeout(connect,2000)};ws.onerror=function(){try{ws.close()}catch(e){}};ws.onmessage=function(e){try{var d=JSON.parse(e.data);if(d.msg){toast(d.msg);setTimeout(function(){el('drawer').classList.remove('open')},2000);return}update(d)}catch(_){}}}
function hex(v){return('00000000'+(v>>>0).toString(16).toUpperCase()).slice(-8)}
function update(d){lastData=Date.now();el('app').classList.remove('stale');var bat=d.bat||0,pct=Math.max(0,Math.min(100,Math.round((bat-10.5)/(12.6-10.5)*100)));text('proto',d.pv);text('batTop',bat.toFixed(1)+'V');text('batMain',bat.toFixed(1)+'V');text('batPct',pct+'%');el('batArc').style.strokeDashoffset=100-pct;el('batArc').style.stroke=bat<10.5?'var(--danger)':(bat<11.5?'var(--warn)':'var(--ok)');text('mode',srcNames[d.src]||String(d.src));text('fresh','实时');estop=!!(d.flags&1);lineOn=!!(d.flags&4);el('joy').classList.toggle('locked',estop);el('btnEstop').classList.toggle('active',estop);text('btnEstop',estop?'解除急停':'急停');el('btnLine').classList.toggle('active',lineOn);text('btnLine',lineOn?'巡线开':'巡线关');el('btnStop').disabled=estop;el('btnLine').disabled=estop;var err=d.err>>>0,lat=d.lat>>>0;el('errBar').classList.toggle('show',err!==0||lat!==0);text('errBar','错误 0x'+hex(err)+'  锁存 0x'+hex(lat));for(var i=0;i<4;i++){var en=!!(d.mask&(1<<i)),valid=!!(d.valid&(1<<i)),fault=!!(err&(1<<(i+1)));el('m'+i).classList.toggle('fault',fault);text('v'+i,en?(valid?'正常':'超时'):'关闭');text('s'+i,(d.spd[i]||0).toFixed(2));text('e'+i,d.enc[i]||0);text('c'+i,(d.cur[i]||0).toFixed(2));text('p'+i,d.pwm[i]||0)}}
function resetJoy(){active=false;joy.classList.remove('dragging');thumb.style.left='calc(50% - 26px)';thumb.style.top='calc(50% - 26px)';lx=0;az=0;send({cmd:'vel',lx:0,az:0});send({cmd:'stop'})}
function moveJoy(x,y,force){if(estop)return;var r=joy.getBoundingClientRect(),maxX=r.width/2-26,maxY=r.height/2-26,dx=x-r.left-r.width/2,dy=y-r.top-r.height/2;dx=Math.max(-maxX,Math.min(maxX,dx));dy=Math.max(-maxY,Math.min(maxY,dy));thumb.style.left=(r.width/2+dx-26)+'px';thumb.style.top=(r.height/2+dy-26)+'px';var dead=12;if(Math.sqrt(dx*dx+dy*dy)<dead){lx=0;az=0}else{lx=-(dy/maxY)*.5*spdScale;az=(dx/maxX)*1.0*spdScale}var now=Date.now();if(force||now-lastSend>=50){lastSend=now;send({cmd:'vel',lx:Number(lx.toFixed(3)),az:Number(az.toFixed(3))})}}
joy.addEventListener('pointerdown',function(e){if(estop)return;active=true;joy.classList.add('dragging');joy.setPointerCapture(e.pointerId);moveJoy(e.clientX,e.clientY,true)});
joy.addEventListener('pointermove',function(e){if(active)moveJoy(e.clientX,e.clientY,false)});
joy.addEventListener('pointerup',resetJoy);joy.addEventListener('pointercancel',resetJoy);
el('btnStop').onclick=function(){resetJoy()};
el('btnEstop').onclick=function(){send({cmd:'estop',v:estop?0:1})};
el('btnLine').onclick=function(){if(!estop)send({cmd:'line',v:lineOn?0:1})};
el('spdScale').oninput=function(){spdScale=parseInt(this.value,10)/100;text('spdVal',this.value+'%')};
el('openSettings').onclick=function(){el('drawer').classList.add('open')};
el('closeSettings').onclick=el('shade').onclick=function(){el('drawer').classList.remove('open')};
function wifiValid(){el('saveWifi').disabled=!el('ssid').value}
el('ssid').oninput=wifiValid;el('pass').oninput=wifiValid;
el('saveWifi').onclick=function(){send({cmd:'config',ssid:el('ssid').value,pass:el('pass').value});toast('正在保存网络配置')};
setInterval(function(){if(Date.now()-lastData>300){el('app').classList.add('stale');text('fresh','超时')}},150);
connect();
</script>
</body>
</html>
)raw";

// 推送遥测 JSON 到 WebSocket 客户端
static void pushTelemetry() {
  if (!g_ws_connected || !g_status_valid) return;
  unsigned long now = millis();
  if (now - g_last_telem_ms < TELEM_INTERVAL_MS) return;
  g_last_telem_ms = now;

  char json[512];
  snprintf(json, sizeof(json),
    "{\"pv\":%u,\"flags\":%u,\"src\":%u,\"mask\":%u,\"valid\":%u,"
    "\"err\":%lu,\"lat\":%lu,\"bat\":%.2f,"
    "\"spd\":[%.2f,%.2f,%.2f,%.2f],"
    "\"enc\":[%ld,%ld,%ld,%ld],"
    "\"cur\":[%.2f,%.2f,%.2f,%.2f],"
    "\"tgt\":[%.2f,%.2f,%.2f,%.2f],"
    "\"pwm\":[%d,%d,%d,%d],"
    "\"lx\":%.2f,\"az\":%.2f}",
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
    g_lx, g_az);
  ws.sendTXT(g_ws_client_id, json);
}

// HTTP 路由：首页
static void handleRoot() {
  http.sendHeader("Cache-Control", "no-cache");
  http.send_P(200, "text/html", HTML_PAGE);
}

// HTTP 路由：状态 JSON（方便调试）
static void handleStatus() {
  char json[256];
  snprintf(json, sizeof(json),
    "{\"pv\":%u,\"bat\":%.2f,\"err\":%lu,\"lat\":%lu,\"src\":%u,"
    "\"wifi\":\"%s\",\"sta_cfg\":%d,\"ws\":%d}",
    g_status.protocol_version,
    (float)g_status.battery_mv / 1000.0f,
    (unsigned long)g_status.error_flags,
    (unsigned long)g_status.latched_error_flags,
    g_status.control_source,
    my_ip.c_str(), sta_configured, g_ws_connected);
  http.send(200, "application/json", json);
}

// ============================================================================
// 8. 初始化
// ============================================================================

void setup() {
  Serial.begin(UART_BAUD);
  // ESP12F 的 UART0 TX/RX 默认在 GPIO1/GPIO3，与 STM32 USART2 交叉连接
  // 如需调试输出，可通过 Serial1 (GPIO2) 或禁用调试

  // WiFi 初始化（AP+STA 智能切换）
  wifiInit();

  // HTTP 服务器
  http.on("/", handleRoot);
  http.on("/status", handleStatus);
  http.begin();

  // WebSocket 服务器
  ws.begin();
  ws.onEvent(wsEvent);

  // 状态 LED（ESP12F 内置 LED 在 GPIO2，低电平亮）
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
}

// ============================================================================
// 9. 主循环
// ============================================================================

void loop() {
  unsigned long now = millis();

  // --- 收 STM32 数据 ---
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (feedRxByte(b)) {
      // 收到有效帧
      uint8_t cmd = rx_frame_buf[3];
      uint8_t cmd_len = rx_frame_buf[2];
      uint8_t *payload = &rx_frame_buf[4];
      uint8_t payload_len = cmd_len - 1;

      if (cmd == CMD_STATUS && parseStatusFrame(payload, payload_len, g_status)) {
        g_status_valid = true;
      }
    }
  }

  // --- 推送遥测 ---
  pushTelemetry();

  // --- 速度指令超时保护 ---
  // 如果 500ms 没有收到 WebSocket 速度指令，自动清零（防失控）
  if (g_vel_active && (now - g_last_vel_ms > 500)) {
    g_vel_active = false;
    uint8_t buf[PROTO_MAX_PAYLOAD + 5];
    uint16_t len = buildVelocityFrame(0, 0, 0, buf, sizeof(buf));
    if (len) sendToSTM32(buf, len);
  }

  // --- HTTP / WebSocket 事件 ---
  wifiMaintain();
  http.handleClient();
  ws.loop();

  // --- 心跳 LED（收到遥测时闪） ---
  if (g_status_valid) {
    digitalWrite(2, (now % 1000) < 100 ? LOW : HIGH);
  } else {
    digitalWrite(2, (now % 2000) < 200 ? LOW : HIGH);
  }
}
