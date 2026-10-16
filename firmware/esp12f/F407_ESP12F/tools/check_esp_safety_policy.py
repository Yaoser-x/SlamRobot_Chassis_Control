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


def ordered(text: str, *markers: str) -> bool:
    offset = 0
    for marker in markers:
        found = text.find(marker, offset)
        if found < 0:
            return False
        offset = found + len(marker)
    return True


def bridge_holds_maintenance(bridge: str) -> bool:
    restore_services = section(bridge, "static void Esp12fFlashBridge_RestoreServices", "static void Esp12fFlashBridge_BeginRestore")
    enable = section(bridge, "uint8_t Esp12fFlashBridge_Enable", "void Esp12fFlashBridge_Disable")
    disable = section(bridge, "void Esp12fFlashBridge_Disable", "uint8_t Esp12fFlashBridge_IsActive")
    begin_restore = section(bridge, "static void Esp12fFlashBridge_BeginRestore", "void Esp12fFlashBridge_Init")
    update = section(bridge, "void Esp12fFlashBridge_Update", "void Esp12fFlashBridge_GetState")
    starting = section(update, "if (bridge_phase == ESP12F_BRIDGE_STARTING)", "if (bridge_phase == ESP12F_BRIDGE_RESTORING)")
    restoring = section(update, "if (bridge_phase == ESP12F_BRIDGE_RESTORING)", "if (bridge_phase != ESP12F_BRIDGE_FORWARDING)")
    return (
        '#include "motion_maintenance_orchestrator.h"' in bridge
        and "AppMotionMaintenance_End();" in bridge
        and ordered(enable, "bridge_phase == ESP12F_BRIDGE_RESTORING", "return 0U;",
                    "bridge_phase != ESP12F_BRIDGE_OFF")
        and ordered(enable, "AppMotionMaintenance_Begin() != APP_MOTION_MAINTENANCE_OK",
                    "return 0U;", "bridge_maintenance_lock_held = 1U;",
                    "bridge_phase = ESP12F_BRIDGE_PREPARING;", "UartBridgeTransport_Stop();",
                    "Esp12fBootControl_EnterDownload();", "bridge_phase = ESP12F_BRIDGE_STARTING;")
        and "Esp12fFlashBridge_ReleaseMaintenance();" not in enable
        and "UartBridgeTransport_Start(" not in enable
        and ordered(begin_restore, "bridge_phase = ESP12F_BRIDGE_RESTORING;",
                    "UartBridgeTransport_Stop();", "Esp12fBootControl_EnterNormal();")
        and "Esp12fFlashBridge_ReleaseMaintenance();" not in begin_restore
        and ordered(restore_services, "Usart1DebugConsole_RestartRx();",
                    "WirelessCommunication_RestartRx();")
        and ordered(disable, "bridge_phase != ESP12F_BRIDGE_RESTORING",
                    "Esp12fFlashBridge_BeginRestore();")
        and ordered(starting, "Esp12fBootControl_Update(PlatformTime_NowMs()) != 0U",
                    "UartBridgeTransport_Start(PlatformTime_NowMs()) != 0U",
                    "bridge_phase = ESP12F_BRIDGE_FORWARDING;", "else",
                    "Esp12fFlashBridge_BeginRestore();")
        and "Esp12fFlashBridge_ReleaseMaintenance();" not in starting
        and ordered(restoring, "Esp12fBootControl_Update(PlatformTime_NowMs()) != 0U",
                    "Esp12fFlashBridge_RestoreServices();", "bridge_phase = ESP12F_BRIDGE_OFF;",
                    "Esp12fFlashBridge_ReleaseMaintenance();")
        and ordered(update, "idle_ms >= ESP12F_FLASH_BRIDGE_IDLE_TIMEOUT_MS",
                    "Esp12fFlashBridge_Disable();")
    )


def main() -> int:
    sketch = SKETCH_PATH.read_text(encoding="utf-8")
    link_policy = (SKETCH_PATH.parent / "esp_link_policy.h").read_text(encoding="utf-8")
    frame_parser = (SKETCH_PATH.parent / "esp_frame_parser.h").read_text(encoding="utf-8")
    esp_source = sketch + "\n" + link_policy + "\n" + frame_parser
    upper_uart = (ROOT / "Service/communication/host_communication_service.c").read_text(encoding="utf-8")
    esp_comm = (ROOT / "Service/communication/wireless_communication_service.c").read_text(encoding="utf-8")
    command_router = (ROOT / "Service/communication/internal/remote_command_dispatcher.c").read_text(encoding="utf-8")
    bridge = (ROOT / "App/adapters/esp12f_flash_bridge.c").read_text(encoding="utf-8")
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
        "PROTO_VERSION       3",
        "REQUIRED_CAPABILITIES 0x1F",
        "controlHandshakeReady",
        "COMMAND_ACK_TIMEOUT_MS 150UL",
        "COMMAND_KEEPALIVE_MS   50UL",
        "g_session_id",
        "g_release_acked",
        "buildGetInfoFrame",
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
        if name == "vel" and "sendVelocityCommand(g_lx, g_az, 1U, true)" not in body:
            errors.append("fresh WebSocket velocity does not advance its v3 sequence")
    if "sendVelocityCommand(g_lx, g_az, 1U, false)" not in section(sketch, "void loop()", "// --- HTTP / WebSocket 事件 ---"):
        errors.append("UART keepalive must reuse the accepted command sequence")
    neutral_body = section(sketch, "static void sendNeutralControl()", "static void releaseOwnerAndStop()")
    release_body = section(sketch, "static void releaseOwnerAndStop()", "// WebSocket 事件处理")
    if "stopChassis();" not in neutral_body or "buildLineCtrlFrame(0U" not in neutral_body:
        errors.append("neutral control does not revoke velocity and line modes")
    if "sendNeutralControl();" not in release_body:
        errors.append("owner release does not revoke velocity and line modes")
    setup_body = section(sketch, "void setup()", "void loop()")
    if "attempt < 3U" not in setup_body or "sendVelocityCommand(0.0f, 0.0f, 0U, false)" not in setup_body:
        errors.append("ESP startup does not send three identical v3 release commands")
    if not bridge_holds_maintenance(bridge):
        errors.append("ESP bridge does not hold the unified maintenance lock")
    if ("UpperProtocol_ParseVersionedFlag" not in command_router
            or "requested != 0U" not in command_router):
        errors.append("shared remote ESTOP routing is not set-only")
    for marker in ("rx_timeout_resets", "uart_errors", "WirelessCommunication_ResetParser();"):
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
