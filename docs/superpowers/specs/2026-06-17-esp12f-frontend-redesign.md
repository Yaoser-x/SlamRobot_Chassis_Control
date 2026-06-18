# ESP12F Web Frontend Redesign

**Date**: 2026-06-17
**Status**: Design approved, pending implementation plan

## 1. Overview

Redesign the ESP12F WiFi bridge web remote control panel. The current frontend is a minimal functional prototype embedded as a PROGMEM HTML string in the Arduino sketch. This redesign targets a modern Material Dark aesthetic with improved information architecture, per-motor telemetry visualization, and WiFi configuration UI.

### Goals

- Modern Material Dark visual style (no emoji, CSS/SVG icons only)
- Per-motor telemetry: M1-M4 encoder counts, speeds, currents (currently only left/right aggregated)
- Visual gauges: SVG battery arc, motor status cards with side-colored borders
- WiFi configuration form with SSID/password save
- Improved joystick: rectangular touch area, trailing shadow particles, snap-back animation
- Status bar with connection state and battery voltage
- Smooth page transitions and data freshness indicators

### Constraints

- Single HTML file embedded as PROGMEM string in `F407_ESP12F.ino`
- No external CDN dependencies (ESP may run in AP mode without internet)
- ESP8266 4MB Flash — no practical size limit for this scope (~15-18KB expected)
- Must work on mobile browsers (touch + mouse)
- Protocol change needed: extend `upper_status_payload_t` for per-motor data

## 2. Layout Architecture

```
+------------------------------------------+
|  Status Bar (36px)                       |
|  [dot] Connected  ·  F407    12.4V  [gear]|
+------------------------------------------+
|  Dashboard Area (upper, compact)         |
|  +-- Battery Arc --+ +-- Control Mode --+ |
|  |    12.4V 85%    | |    ESP12F        | |
|  +-----------------+ +------------------+ |
|  +----- M1 ------+ +----- M2 ------+     |
|  | 1234 enc       | | 1240 enc       |    |
|  | 0.32 m/s       | | 0.31 m/s       |    |
|  | 0.45 A         | | 0.42 A         |    |
|  +---------------+ +---------------+     |
|  +----- M3 ------+ +----- M4 ------+     |
|  | 1230 enc       | | 1228 enc       |    |
|  | 0.33 m/s       | | 0.32 m/s       |    |
|  | 0.44 A         | | 0.43 A         |    |
|  +---------------+ +---------------+     |
|  Error bar (conditional, red, auto-hide) |
+------------------------------------------+
|  Joystick Control Area (lower, dominant) |
|  +-- Crosshair Guide Lines (subtle) --+  |
|  |                                    |  |
|  |           [thumb 52px]             |  |
|  |                                    |  |
|  +-- 280 x 200px touch area ---------+  |
|  Speed Scale: [====o====] 60%          |
|  [     STOP     ] [E-STOP] [Line]      |
+------------------------------------------+
```

**Settings page**: slides in from right (250ms ease-out) when gear icon tapped.
Semi-transparent backdrop + 320px wide panel with form fields and close button.

No bottom tab bar — single main page with settings overlay.

## 3. Visual Design System

### 3.1 Color Palette (CSS Custom Properties)

| Token | Value | Usage |
|-------|-------|-------|
| `--bg` | `#0D1117` | Page background |
| `--surface` | `#161B22` | Cards, joystick area |
| `--surface-elevated` | `#1C2333` | Settings panel, overlays |
| `--primary` | `#58A6FF` | Primary actions, left-side motors |
| `--accent` | `#00E5FF` | Joystick thumb, highlights, right-side motors |
| `--success` | `#3FB950` | Connected state, line on |
| `--warning` | `#D29922` | Battery mid, E-STOP inactive |
| `--danger` | `#F85149` | STOP, E-STOP active, overcurrent, errors |
| `--text-primary` | `#F0F6FC` | Primary text |
| `--text-secondary` | `#8B949E` | Secondary text, labels |
| `--border` | `#30363D` | Dividers, card borders |

### 3.2 Typography

- Font stack: `system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif`
- Monospace (data values): `"SF Mono", "Cascadia Code", "Consolas", monospace`
- Scale: title 18px bold, card label 10px uppercase tracked, data value 16px bold, body 13px

### 3.3 Spacing & Shape

- Page padding: 12px
- Card padding: 10px 12px
- Card gap: 8px
- Border radius: 10px (cards), 6px (buttons), 50% (thumb, status dot)
- Button height: 48px (STOP), 40px (E-STOP/Line)

## 4. Component Specifications

### 4.1 Status Bar

- Height: 36px, full width
- Left: 8px green/red dot (CSS border-radius 50%) + connection text
- Center: WiFi AP name (truncated)
- Right: battery voltage in monospace + SVG gear icon (24x24, `--text-secondary`, no emoji)
- Background: `--bg`, bottom border `--border`

### 4.2 Joystick

- 280 x 200px rectangular touch area, centered, `--surface` background
- Subtle crosshair: horizontal + vertical center lines at `--border` 30% opacity
- Thumb: 52px circle, radial gradient `--accent` to `--primary`, `box-shadow: 0 0 24px rgba(0,229,255,0.4)`
- Trailing particles: 8 CSS box-shadow layers at decreasing opacity, no JS animation
- Snap-back: `transition: left 200ms ease-out, top 200ms ease-out`
- Dead zone: 12px radius from center
- Clamp: thumb cannot exceed area bounds; max speed sent when at edge

### 4.3 Motor Cards (4x, 2x2 grid)

Each card shows three rows:

| Row | Label | Format | Example |
|-----|-------|--------|---------|
| Enc | ENC | monospace int32 | `1234` |
| Speed | m/s | decimal 2dp | `0.32` |
| Current | A | decimal 2dp | `0.45` |

- Left edge 3px colored border: M1/M2 = `--primary` (blue), M3/M4 = `--accent` (cyan)
- Background: `--surface`, header shows motor ID (M1-M4) in 11px uppercase
- Overcurrent: border-color switches to `--danger` with 500ms pulse animation
- Data freshness: values flash highlight briefly on update (250ms transition)

### 4.4 Battery Gauge (SVG)

- SVG semi-circular arc, ~210 sweep angle
- Track: `--border` stroke, Value: gradient stroke
- Color thresholds: green > 11.5V, yellow 10.5-11.5V, red < 10.5V
- Center: large voltage value (18px bold) + percentage below (12px secondary)
- Dimensions: ~120x70px viewBox

### 4.5 Buttons

**STOP**:
- Full width, `--danger` background, white text, 48px height
- Border-radius: 10px, font-weight: 600
- Active: `transform: scale(0.96)`, 100ms transition

**E-STOP** (toggle):
- Inactive: `--warning` background, label "E-STOP"
- Active: `--danger` background with pulse animation (`box-shadow` 0-50% opacity cycle), label "UNSTOP"
- ~48% width (shares row with Line button)

**Line** (toggle):
- Off: `--border` background, `--text-secondary` text, label "Line OFF"
- On: `--success` background, white text, label "Line ON"
- ~48% width

### 4.6 Speed Slider

- Custom-styled `<input type="range">`
- Track: `--surface` background, 4px height, border-radius 2px
- Filled track: `--primary`
- Thumb: 20px circle, `--primary` background, white border, `--accent` glow on active
- Right label: current percentage in monospace

### 4.7 Settings Page

- Overlay: fixed position, semi-transparent black backdrop (rgba(0,0,0,0.6))
- Panel: 320px wide, right-aligned, `--surface-elevated` background
- Slide-in: `transform: translateX(100%)` to `translateX(0)`, 250ms ease-out
- Form fields: `--surface` background inputs, 12px padding, `--border` border, 8px radius
- Floating labels: transition from placeholder position to above-input on focus/filled (pure CSS `:focus-within`)
- Save button: `--primary` background, full panel width minus padding
- Footer: current WiFi connection info card (SSID, IP address)
- Close: X button (SVG) top-right + tap backdrop to close

### 4.8 Control Mode Indicator

- Small card alongside battery gauge
- Shows current active control source: "STOP", "ESP12F", "PS2", "RPI", "LINE", "DEBUG"
- Color-coded label background per source

### 4.9 Error Bar

- Conditional display: shown only when `error_flags != 0`
- `--danger` background at 15% opacity, red left border
- Shows hex error code in monospace
- Auto-hides when error clears

## 5. Interaction & State Handling

### 5.1 Joystick

- Touch: `touchstart` activate, `touchmove` update position + send vel, `touchend` snap-back + send zero
- Mouse: `mousedown`/`mousemove`/`mouseup` equivalent
- 12px dead zone from center — ignore micro-movements
- Velocity command sent on every move event (no throttling — ESP firmware handles 500ms timeout)
- Snap-back sends `{"cmd":"vel","lx":"0.000","az":"0.000"}` then `{"cmd":"stop"}`

### 5.2 Telemetry Refresh

- WebSocket pushes at 100ms intervals
- On message: parse JSON, update all DOM values
- Value change flash: set CSS class for 250ms, auto-remove via `transitionend`
- Stale data: if no message for 300ms, add `.stale` class to dashboard (reduces opacity to 0.4)
- Connection lost triggers stale immediately, restored on reconnect + first message

### 5.3 Connection States

| State | Dot color | Text |
|-------|-----------|------|
| Connected | `--success` static | "Connected" |
| Disconnected | `--danger` pulse (1s cycle) | "Reconnecting..." |
| First connect | `--success` ripple animation (single shot) | "Connected" |

WebSocket auto-reconnect: 2s interval on close/error.

### 5.4 Error Indication

- Per-motor overcurrent: card left border pulses red 500ms cycle
- Global error: error bar appears below motor grid with hex code
- E-STOP active: joystick area dimmed + overlaid with "ESTOP ACTIVE" text, all control buttons disabled

### 5.5 Settings

- Save button disabled until both SSID and password fields non-empty
- On save: send `{"cmd":"config","ssid":"...","pass":"..."}`
- On success response: show "Saved. Reboot to apply." toast, auto-close panel after 2s
- Close via X button, backdrop tap, or auto-close after save

## 6. Protocol Extension

### 6.1 Status Payload Changes

`upper_status_payload_t` currently 45 bytes. Extended to 77 bytes:

```c
typedef struct {
  // Existing (37 bytes)
  float    left_speed;         // 4B
  float    right_speed;        // 4B
  int32_t  left_encoder;       // 4B
  int32_t  right_encoder;      // 4B
  float    battery_voltage;    // 4B
  float    left_current;       // 4B
  float    right_current;      // 4B
  int16_t  imu_accel[3];       // 6B
  int16_t  imu_gyro[3];        // 6B
  uint32_t error_flags;        // 4B
  uint8_t  control_mode;       // 1B
  // padding                    // 0B (was implicit)
  // New (32 bytes)
  int32_t  encoder_count[4];   // 16B — M1-M4 individual
  float    motor_current[4];   // 16B — M1-M4 individual
} upper_status_payload_t;
// New total: 77 bytes
// Update UPPER_PROTOCOL_STATUS_PAYLOAD_LEN to 77U
```

Keep existing left/right fields for backward compatibility (OLED display and debug console also consume this struct).

### 6.2 WebSocket Telemetry JSON

Add per-motor fields to the JSON pushed every 100ms:

```json
{
  "ls": 0.32, "rs": 0.31,
  "le": 1234, "re": 1240,
  "bat": 12.4,
  "lc": 0.45, "rc": 0.44,
  "err": 0, "src": 3,
  "lx": 0.30, "az": 0.00,
  "enc": [1234, 1240, 1230, 1228],
  "cur": [0.45, 0.42, 0.44, 0.43]
}
```

JSON buffer in ESP12F increased from 256 to 384 bytes to accommodate new fields.

## 7. File Change Summary

| File | Change |
|------|--------|
| `App/protocol/upper_protocol.h` | Extend `upper_status_payload_t`, update `STATUS_PAYLOAD_LEN` 45→77 |
| `App/protocol/upper_protocol.c` | Update `BuildStatusPayload` to serialize new fields |
| `BSP/esp12f/esp12f_comm.c` | Populate `encoder_count[]` and `motor_current[]` from driver state |
| `firmware/esp12f/F407_ESP12F/F407_ESP12F.ino` | Replace `HTML_PAGE` with new design; update `parseStatusFrame`; update `pushTelemetry`; increase JSON buffer to 384B |

## 8. Edge Cases

- **Protocol mismatch**: ESP12F firmware flashed but STM32 not updated → new fields will be garbage. Frontend should check array length before accessing `enc[]`/`cur[]`, fall back to showing left/right only.
- **Large encoder values**: int32 can hold ~2 billion counts. Display without overflow. Consider showing as "1.2K" if > 9999 for compactness.
- **Current sensor noise**: STM32 already applies EMA filter (alpha=0.25). Frontend displays raw value, no additional filtering.
- **Concurrent touch**: multiple fingers on joystick — use first touch only (existing behavior, keep).
- **Settings page open during disconnect**: auto-close settings, show reconnecting state.

## 9. Verification

1. Build STM32 firmware: `cmake --preset Debug && cmake --build --preset Debug` — verify no compile errors from protocol changes
2. Build ESP12F firmware: Arduino IDE compile — verify HTML fits in memory (check `.bin` size)
3. Flash both and test:
   - Phone connects to `F407_Chassis` AP, opens `192.168.4.1`
   - Confirm dashboard loads with Material Dark theme
   - Joystick responds to touch, sends velocity commands
   - M1-M4 cards show individual encoder/speed/current values
   - Battery arc updates and changes color at thresholds
   - E-STOP toggles, locks joystick
   - Line toggles, green/grey
   - Settings gear opens panel, WiFi form works, save sends config
   - Disconnect → reconnect → verify status indicators
4. Host tests: `ctest --test-dir build/host-tests-ninja --output-on-failure` — verify protocol serialization/deserialization still passes
