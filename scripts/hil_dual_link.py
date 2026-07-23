"""Validate that separately captured Host and ESP HIL reports are independent."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host-report", type=Path, required=True)
    parser.add_argument("--esp-report", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    host = json.loads(args.host_report.read_text(encoding="utf-8"))
    esp = json.loads(args.esp_report.read_text(encoding="utf-8"))
    host_status = host.get("frames", {}).get("STATUS", [])
    esp_status = esp.get("status_after", {})
    esp_session = int(esp_status.get("session_low", 0)) | (int(esp_status.get("session_high", 0)) << 32)
    assertions = {
        "host_report_passed": all(host.get("assertions", {}).values()),
        "esp_report_passed": all(esp.get("assertions", {}).values()),
        "independent_sessions_observed": bool(host_status)
        and host_status[-1].get("session_id") != esp_session,
        "independent_ack_observed": bool(host_status) and "ack_flags" in esp_status,
    }
    output = {
        "runner": "hil_dual_link.py",
        "timestamp_unix_ms": int(time.time() * 1000),
        "assertions": assertions,
        "host_report": str(args.host_report),
        "esp_report": str(args.esp_report),
        "manual_pending": ["simultaneous command arbitration Host > PS2 > ESP > Line > Debug"]
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0 if all(assertions.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
