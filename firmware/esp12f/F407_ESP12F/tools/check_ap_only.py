#!/usr/bin/env python3
"""Validate that the ESP12F firmware exposes only its fixed access point."""

from __future__ import annotations

import sys
from pathlib import Path


SKETCH_PATH = Path(__file__).resolve().parent.parent / "F407_ESP12F.ino"

REQUIRED_MARKERS = (
    "WiFi.mode(WIFI_AP);",
    "WiFi.softAP(AP_SSID, AP_PASS);",
    'static String my_ip;',
)

FORBIDDEN_MARKERS = (
    "#include <EEPROM.h>",
    "WIFI_AP_STA",
    "WIFI_NONE_SLEEP",
    "WiFi.begin(",
    "WiFi.status(",
    "WiFi.localIP(",
    "EEPROM.",
    "STA_TIMEOUT_MS",
    "STA_RECONNECT_INTERVAL_MS",
    "sta_ssid",
    "sta_pass",
    "sta_configured",
    "wifiMaintain",
    '"sta_cfg"',
    r'{\"cmd\":\"config\"',
    'id="openSettings"',
    'id="drawer"',
    'id="saveWifi"',
    ".drawer{",
    ".toast{",
    ".icon{",
)


def main() -> int:
    sketch = SKETCH_PATH.read_text(encoding="utf-8")
    errors: list[str] = []

    for marker in REQUIRED_MARKERS:
        if marker not in sketch:
            errors.append(f"missing AP-only marker: {marker}")

    for marker in FORBIDDEN_MARKERS:
        if marker in sketch:
            errors.append(f"forbidden STA/config marker remains: {marker}")

    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1

    print("AP-only firmware check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
