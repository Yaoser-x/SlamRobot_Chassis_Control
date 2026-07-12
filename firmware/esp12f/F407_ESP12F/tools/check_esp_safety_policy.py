#!/usr/bin/env python3
"""Validate ESP first-boot configuration, owner lease, and set-only ESTOP policy."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
SKETCH_PATH = Path(__file__).resolve().parent.parent / "F407_ESP12F.ino"


def section(text: str, start: str, end: str) -> str:
    begin = text.find(start)
    finish = text.find(end, begin + len(start)) if begin >= 0 else -1
    if begin < 0 or finish < 0:
        return ""
    return text[begin:finish]


def main() -> int:
    sketch = SKETCH_PATH.read_text(encoding="utf-8")
    link_policy = (SKETCH_PATH.parent / "esp_link_policy.h").read_text(encoding="utf-8")
    frame_parser = (SKETCH_PATH.parent / "esp_frame_parser.h").read_text(encoding="utf-8")
    esp_source = sketch + "\n" + link_policy + "\n" + frame_parser
    upper_uart = (ROOT / "App/protocol/upper_uart.c").read_text(encoding="utf-8")
    esp_comm = (ROOT / "App/protocol/esp12f_comm.c").read_text(encoding="utf-8")
    bridge = (ROOT / "BSP/esp12f/esp12f_flash_bridge.c").read_text(encoding="utf-8")
    errors: list[str] = []

    required = (
        "#include <EEPROM.h>",
        'SETUP_AP_SSID     "F407_Chassis_Setup"',
        "CONFIG_MAGIC",
        "CONFIG_VERSION",
        "CONFIG_PASSWORD_MIN_LEN 8",
        "CONFIG_PASSWORD_MAX_LEN 63",
        "configCrc32",
        "loadConfig",
        "saveConfig",
        "EEPROM.commit()",
        "if (g_config_valid)",
        "ws.begin();",
        "NO_OWNER 0xFF",
        "OWNER_LEASE_MS 500",
        "OWNER_HEARTBEAT_MS 200",
        'cmd.startsWith("{\\"cmd\\":\\"claim\\"")',
        'cmd.startsWith("{\\"cmd\\":\\"heartbeat\\"")',
        'cmd.startsWith("{\\"cmd\\":\\"clearfault\\"")',
        "buildClearFaultFrame",
        "CMD_CLEAR_FAULT",
        "faultNames",
        "btnClearFault",
        "clearPendingLatched",
        "content:attr(data-lock-reason)",
        "dataset.lockReason",
        "只读模式",
        "clientIsOwner(num)",
        "releaseOwnerAndStop",
        "buildLineCtrlFrame(0U",
        "ws.broadcastTXT",
        "buildEstopFrame(1U",
        "请本地解除",
        "sendNeutralControl();",
        "EspLinkPolicy_Update",
        "ESP_LINK_STATUS_TIMEOUT_MS 500UL",
        "ESP_FRAME_INTERBYTE_TIMEOUT_MS 100UL",
        "status_age_ms",
        '\\"online\\":%s',
        "Serial.hasOverrun()",
        "Serial.hasRxError()",
        "g_link_policy",
        "EspFrameParser_OnUartError",
    )
    forbidden = (
        "#define AP_PASS",
        "12345678",
        "g_ws_client_id",
        "buildEstopFrame(v",
        "estop?0:1",
        "ControlManager_SetEmergencyStop(payload[0])",
        "g_config_valid = true;",
    )

    for marker in required:
        if marker not in esp_source:
            errors.append(f"missing ESP safety marker: {marker}")
    for marker in forbidden:
        if marker in sketch or marker in upper_uart or marker in esp_comm:
            errors.append(f"forbidden unsafe marker remains: {marker}")

    handlers = (
        ("clearfault", section(sketch, 'else if (cmd.startsWith("{\\"cmd\\":\\"clearfault\\""))',
                               'else if (cmd.startsWith("{\\"cmd\\":\\"vel\\""))')),
        ("vel", section(sketch, 'else if (cmd.startsWith("{\\"cmd\\":\\"vel\\""))',
                        'else if (cmd.startsWith("{\\"cmd\\":\\"stop\\""))')),
        ("stop", section(sketch, 'else if (cmd.startsWith("{\\"cmd\\":\\"stop\\""))',
                         'else if (cmd.startsWith("{\\"cmd\\":\\"line\\""))')),
        ("line", section(sketch, 'else if (cmd.startsWith("{\\"cmd\\":\\"line\\""))',
                         "    break;\n  }")),
    )
    for name, body in handlers:
        if "if (!clientIsOwner(num))" not in body:
            errors.append(f"{name} handler is not owner guarded")
    neutral_body = section(sketch, "static void sendNeutralControl()", "static void releaseOwnerAndStop()")
    release_body = section(sketch, "static void releaseOwnerAndStop()", "// WebSocket 事件处理")
    if "stopChassis();" not in neutral_body or "buildLineCtrlFrame(0U" not in neutral_body:
        errors.append("neutral control does not revoke velocity and line modes")
    if "sendNeutralControl();" not in release_body:
        errors.append("owner release does not revoke velocity and line modes")
    setup_body = section(sketch, "void setup()", "void loop()")
    if "sendNeutralControl();" not in setup_body:
        errors.append("ESP startup does not actively revoke stale STM32 motion modes")
    bridge_enable = section(bridge, "uint8_t Esp12fFlashBridge_Enable", "void Esp12fFlashBridge_Disable")
    bridge_disable = section(bridge, "void Esp12fFlashBridge_Disable", "uint8_t Esp12fFlashBridge_IsActive")
    bridge_update = section(bridge, "void Esp12fFlashBridge_Update", "void Esp12fFlashBridge_OnTxCplt")
    bridge_rx_failure = section(bridge_enable, "if (rx_ok == 0U)", "return 1U;")
    begin_index = bridge_enable.find("ChassisMaintenance_Begin()")
    active_index = bridge_enable.find("bridge_state.active = 1U")
    failure_end_index = bridge_rx_failure.find("ChassisMaintenance_End();")
    failure_return_index = bridge_rx_failure.find("return 0U;")
    disable_active_index = bridge_disable.find("bridge_state.active = 0U")
    disable_end_index = bridge_disable.rfind("ChassisMaintenance_End();")
    idle_condition_index = bridge_update.find("idle_ms >= ESP12F_FLASH_BRIDGE_IDLE_TIMEOUT_MS")
    idle_disable_index = bridge_update.find("Esp12fFlashBridge_Disable();", idle_condition_index)
    if ('#include "chassis_maintenance.h"' not in bridge or
            "bridge_maintenance_lock_held" not in bridge or
            begin_index < 0 or active_index < 0 or begin_index > active_index or
            failure_end_index < 0 or failure_return_index < 0 or
            failure_end_index > failure_return_index or
            disable_active_index < 0 or disable_end_index < disable_active_index or
            idle_condition_index < 0 or idle_disable_index < idle_condition_index):
        errors.append("ESP bridge does not hold the unified maintenance lock")
    if "UpperProtocol_RemoteEstopSetRequested" not in upper_uart:
        errors.append("upper UART remote ESTOP is not set-only")
    if "UpperProtocol_RemoteEstopSetRequested" not in esp_comm:
        errors.append("STM32 ESP remote ESTOP is not set-only")
    for marker in ("rx_timeout_resets", "uart_errors", "Esp12fComm_ResetParser();"):
        if marker not in esp_comm:
            errors.append(f"STM32 ESP recovery marker missing: {marker}")
    loop_body = section(sketch, "void loop()", "// --- 推送遥测 ---")
    uart_error_index = loop_body.find("Serial.hasOverrun()")
    consume_index = loop_body.find("if (EspFrameParser_Feed", uart_error_index + 1)
    if uart_error_index < 0 or consume_index < 0 or uart_error_index > consume_index:
        errors.append("ESP UART hardware errors are not handled before consuming RX bytes")

    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1
    print("ESP safety policy check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
