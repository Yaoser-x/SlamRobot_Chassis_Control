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
#define AP_SSID           "F407_Chassis"
#define AP_PASS           "12345678"
#define WEB_SOCKET_PORT   81
#define HTTP_PORT         80
#define EEPROM_SIZE       128
#define EEPROM_MAGIC      0xA5
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_SSID_ADDR  4
#define EEPROM_PASS_ADDR  36      // 4 + 32
#define MAX_SSID_LEN      32
#define MAX_PASS_LEN      32

// 底盘控制范围
#define MAX_LINEAR_MPS    0.5f    // 最大线速度 m/s
#define MAX_ANGULAR_RPS   1.0f    // 最大角速度 rad/s

// 遥测推送间隔（ms）
#define TELEM_INTERVAL_MS 100

// 底盘状态（解析自 STM32 status 帧）
struct ChassisStatus {
  float    left_speed;
  float    right_speed;
  int32_t  left_encoder;
  int32_t  right_encoder;
  float    battery_voltage;
  float    left_current;
  float    right_current;
  int16_t  imu_accel[3];
  int16_t  imu_gyro[3];
  uint32_t error_flags;
  uint8_t  control_mode;
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
#define PROTO_MAX_PAYLOAD   64
#define CMD_SET_VELOCITY    0x01
#define CMD_ESTOP           0x02
#define CMD_LINE_CTRL       0x03
#define CMD_STATUS          0x81
#define VELOCITY_PAYLOAD_LEN 10
#define STATUS_PAYLOAD_LEN  45

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

  auto readFloat = [&]() -> float {
    uint32_t raw = payload[off] | ((uint32_t)payload[off+1] << 8) |
                   ((uint32_t)payload[off+2] << 16) | ((uint32_t)payload[off+3] << 24);
    off += 4;
    float v; memcpy(&v, &raw, 4); return v;
  };
  auto readI32 = [&]() -> int32_t {
    int32_t raw = (int32_t)(payload[off] | ((uint32_t)payload[off+1] << 8) |
                   ((uint32_t)payload[off+2] << 16) | ((uint32_t)payload[off+3] << 24));
    off += 4; return raw;
  };
  auto readI16 = [&]() -> int16_t {
    int16_t raw = (int16_t)(payload[off] | ((uint16_t)payload[off+1] << 8));
    off += 2; return raw;
  };

  s.left_speed      = readFloat();
  s.right_speed     = readFloat();
  s.left_encoder    = readI32();
  s.right_encoder   = readI32();
  s.battery_voltage = readFloat();
  s.left_current    = readFloat();
  s.right_current   = readFloat();
  for (int i = 0; i < 3; i++) s.imu_accel[i] = readI16();
  for (int i = 0; i < 3; i++) s.imu_gyro[i]  = readI16();
  s.error_flags     = (uint32_t)(payload[off] | ((uint32_t)payload[off+1] << 8) |
                       ((uint32_t)payload[off+2] << 16) | ((uint32_t)payload[off+3] << 24));
  off += 4;
  s.control_mode    = payload[off];
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
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  my_ip = WiFi.softAPIP().toString();
  wifi_connected = true;
}

static void wifiInit() {
  eepromLoad();
  WiFi.persistent(false);

  if (sta_configured) {
    // 尝试 STA 模式
    WiFi.mode(WIFI_STA);
    WiFi.begin(sta_ssid, sta_pass);

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
      return;
    }
  }

  // STA 失败或未配置 → AP 模式
  wifiStartAP();
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
    String cmd = String((const char *)payload);

    if (cmd.startsWith("{\"cmd\":\"vel\"")) {
      // {"cmd":"vel","lx":0.30,"az":0.00}
      int lxIdx = cmd.indexOf("\"lx\":");
      int azIdx = cmd.indexOf("\"az\":");
      if (lxIdx > 0 && azIdx > 0) {
        g_lx = cmd.substring(lxIdx + 5).toFloat();
        g_az = cmd.substring(azIdx + 5).toFloat();
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
        ws.sendTXT(num, "{\"msg\":\"WiFi config saved. Reboot to apply.\"}");
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
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no,
  viewport-fit=cover,maximum-scale=1">
<title>F407 Chassis</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#1a1a2e;color:#e0e0e0;font-family:system-ui,sans-serif;
  display:flex;flex-direction:column;align-items:center;min-height:100vh;
  padding:10px;user-select:none;-webkit-user-select:none;touch-action:none}
h1{font-size:18px;margin:4px 0;color:#00d2ff}
#status{font-size:11px;color:#888;margin-bottom:6px;text-align:center}
#joystick{width:200px;height:200px;background:#16213e;border-radius:50%;
  border:2px solid #0f3460;position:relative;margin:8px 0}
#knob{width:56px;height:56px;background:radial-gradient(circle,#00d2ff,#0077b6);
  border-radius:50%;position:absolute;top:72px;left:72px;
  box-shadow:0 0 20px rgba(0,210,255,.5)}
.row{display:flex;gap:8px;margin:4px 0;flex-wrap:wrap;justify-content:center}
.btn{padding:10px 14px;border:none;border-radius:8px;font-size:14px;
  font-weight:600;cursor:pointer;min-width:60px}
.btn-stop{background:#e63946;color:#fff}
.btn-estop{background:#ff6b00;color:#fff}
.btn-line{background:#2a9d8f;color:#fff}
.btn-line.off{background:#555;color:#999}
#telem{font-size:11px;color:#aaa;margin-top:8px;width:100%;max-width:320px;
  display:grid;grid-template-columns:1fr 1fr;gap:2px 12px}
#telem span{color:#00d2ff}
.slider-row{display:flex;align-items:center;gap:6px;margin:2px 0;font-size:12px}
.slider-row input{flex:1;max-width:140px}
</style>
</head>
<body>
<h1>F407 Chassis</h1>
<div id="status">Connecting...</div>
<div id="joystick"><div id="knob"></div></div>
<div class="row">
  <button class="btn btn-stop" id="btnStop">STOP</button>
  <button class="btn btn-estop" id="btnEstop">E-STOP</button>
  <button class="btn btn-line off" id="btnLine">巡线</button>
</div>
<div class="slider-row">
  <label>Max spd:</label>
  <input type="range" id="spdScale" min="10" max="100" value="60">
  <span id="spdVal">60%</span>
</div>
<div id="telem">
  <div>Left: <span id="tls">0.00</span> m/s</div>
  <div>Right: <span id="trs">0.00</span> m/s</div>
  <div>Battery: <span id="tbat">0.0</span> V</div>
  <div>L-Enc: <span id="tle">0</span></div>
  <div>R-Enc: <span id="tre">0</span></div>
  <div>Err: <span id="terr">0</span></div>
  <div>Cmd Lx: <span id="tlx">0.00</span></div>
  <div>Cmd Az: <span id="taz">0.00</span></div>
</div>
<script>
var ws,knob,joystick,active=false,knobR=28,joyR=100,knobCX=100,knobCY=100;
var lineOn=false,spdScale=0.6,estopActive=false;

function connect(){
  var ip=location.hostname;
  ws=new WebSocket('ws://'+ip+':81');
  ws.onopen=function(){document.getElementById('status').textContent='Connected | '+ip};
  ws.onclose=function(){document.getElementById('status').textContent='Disconnected';setTimeout(connect,2000)};
  ws.onmessage=function(e){
    try{
      var d=JSON.parse(e.data);
      document.getElementById('tls').textContent=d.ls.toFixed(2);
      document.getElementById('trs').textContent=d.rs.toFixed(2);
      document.getElementById('tbat').textContent=d.bat.toFixed(1);
      document.getElementById('tle').textContent=d.le;
      document.getElementById('tre').textContent=d.re;
      document.getElementById('terr').textContent=d.err;
      document.getElementById('tlx').textContent=d.lx.toFixed(2);
      document.getElementById('taz').textContent=d.az.toFixed(2);
    }catch(_){}
  };
}

function send(cmd){if(ws&&ws.readyState==1)ws.send(JSON.stringify(cmd))}

function updateJoystick(tx,ty){
  var rect=joystick.getBoundingClientRect();
  var cx=tx-rect.left-rect.width/2,cy=ty-rect.top-rect.height/2;
  var dist=Math.sqrt(cx*cx+cy*cy);
  var maxR=joyR-knobR;
  if(dist>maxR){cx=cx/dist*maxR;cy=cy/dist*maxR}
  knob.style.left=(knobCX+cx-knobR)+'px';
  knob.style.top=(knobCY+cy-knobR)+'px';
  var lx=-(cy/maxR)*0.5*spdScale;
  var az=(cx/maxR)*1.0*spdScale;
  send({cmd:'vel',lx:lx.toFixed(3),az:az.toFixed(3)});
}

function resetJoystick(){
  knob.style.left='72px';knob.style.top='72px';
  send({cmd:'vel',lx:'0.000',az:'0.000'});
}

knob=document.getElementById('knob');
joystick=document.getElementById('joystick');

joystick.addEventListener('touchstart',function(e){active=true;updateJoystick(e.touches[0].clientX,e.touches[0].clientY)});
joystick.addEventListener('touchmove',function(e){e.preventDefault();if(active)updateJoystick(e.touches[0].clientX,e.touches[0].clientY)});
joystick.addEventListener('touchend',function(){active=false;resetJoystick()});
joystick.addEventListener('mousedown',function(e){active=true;updateJoystick(e.clientX,e.clientY)});
joystick.addEventListener('mousemove',function(e){if(active)updateJoystick(e.clientX,e.clientY)});
joystick.addEventListener('mouseup',function(){active=false;resetJoystick()});

document.getElementById('btnStop').addEventListener('click',function(){resetJoystick();send({cmd:'stop'})});
document.getElementById('btnEstop').addEventListener('click',function(){
  estopActive=!estopActive;
  this.style.background=estopActive?'#ff0000':'#ff6b00';
  this.textContent=estopActive?'UNSTOP':'E-STOP';
  send({cmd:'estop',v:estopActive?1:0});
});
document.getElementById('btnLine').addEventListener('click',function(){
  lineOn=!lineOn;
  this.classList.toggle('off',!lineOn);
  this.textContent=lineOn?'巡线:ON':'巡线:OFF';
  send({cmd:'line',v:lineOn?1:0});
});
document.getElementById('spdScale').addEventListener('input',function(){
  spdScale=parseInt(this.value)/100;
  document.getElementById('spdVal').textContent=this.value+'%';
});

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

  char json[256];
  snprintf(json, sizeof(json),
    "{\"ls\":%.2f,\"rs\":%.2f,\"le\":%d,\"re\":%d,"
    "\"bat\":%.1f,\"lc\":%.2f,\"rc\":%.2f,"
    "\"err\":%u,\"src\":%u,\"lx\":%.2f,\"az\":%.2f}",
    g_status.left_speed, g_status.right_speed,
    g_status.left_encoder, g_status.right_encoder,
    g_status.battery_voltage,
    g_status.left_current, g_status.right_current,
    g_status.error_flags, g_status.control_mode,
    g_lx, g_az);
  ws.sendTXT(g_ws_client_id, json);
}

// HTTP 路由：首页
static void handleRoot() {
  http.sendHeader("Cache-Control", "no-cache");
  http.send(200, "text/html", FPSTR(HTML_PAGE));
}

// HTTP 路由：状态 JSON（方便调试）
static void handleStatus() {
  char json[256];
  snprintf(json, sizeof(json),
    "{\"ls\":%.2f,\"rs\":%.2f,\"bat\":%.1f,\"err\":%u,\"src\":%u,"
    "\"wifi\":\"%s\",\"sta_cfg\":%d,\"ws\":%d}",
    g_status.left_speed, g_status.right_speed,
    g_status.battery_voltage,
    g_status.error_flags, g_status.control_mode,
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

      if (cmd == CMD_STATUS && payload_len == STATUS_PAYLOAD_LEN) {
        parseStatusFrame(payload, payload_len, g_status);
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
  http.handleClient();
  ws.loop();

  // --- 心跳 LED（收到遥测时闪） ---
  if (g_status_valid) {
    digitalWrite(2, (now % 1000) < 100 ? LOW : HIGH);
  } else {
    digitalWrite(2, (now % 2000) < 200 ? LOW : HIGH);
  }
}
