#!/usr/bin/env python3
"""Run a read-only HIL smoke test through the USART1 debug console."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import re
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:  # pragma: no cover - user environment check
    serial = None


COMMANDS = ("version", "status", "i2cscan", "imutest", "espflash status")
COMMAND_MARKERS = {
    "version": "version fw=", "status": "POST done=", "i2cscan": "I2C",
    "imutest": "bmi270", "espflash status": "ESPFLASH",
}


def read_until_idle(port: "serial.Serial", timeout_s: float, idle_s: float = 0.3) -> str:
    deadline = time.monotonic() + timeout_s
    last_data = time.monotonic()
    chunks: list[bytes] = []
    while time.monotonic() < deadline:
        data = port.read(port.in_waiting or 1)
        if data:
            chunks.append(data)
            last_data = time.monotonic()
        elif chunks and (time.monotonic() - last_data) >= idle_s:
            break
    return b"".join(chunks).decode("utf-8", errors="replace")


def send_command(port: "serial.Serial", command: str, timeout_s: float) -> str:
    port.write((command + "\r\n").encode("ascii"))
    port.flush()
    return read_until_idle(port, timeout_s)


def build_report(port: str, baud: int, boot_log: str,
                 command_results: list[dict[str, str]], timestamp: str) -> dict[str, object]:
    text = boot_log + "\n" + "\n".join(item["response"] for item in command_results)
    commands = []
    for item in command_results:
        response = item["response"]
        marker = COMMAND_MARKERS[item["command"]]
        assertions = [
            {"name": "response_nonempty", "passed": bool(response.strip())},
            {"name": f"contains:{marker}", "passed": marker.lower() in response.lower()},
        ]
        commands.append({"command": item["command"], "response_summary": response[-500:],
                         "assertions": assertions,
                         "passed": all(assertion["passed"] for assertion in assertions)})
    required = ("POST:", "POST done=", "PARAM ", "version fw=")
    assertions = [{"name": f"contains:{needle}", "passed": needle in text} for needle in required]
    assertions.append({"name": "imu_probe", "passed": "bmi270 probe failed" not in text})
    identity_match = re.search(
        r"version fw=(?P<version>\S+) sha=(?P<sha>\S+) build=(?P<build>\S+) "
        r"protocol=(?P<protocol>\d+) param=(?P<param>\d+) diagnostic=(?P<diagnostic>\d+)", text)
    passed = all(item["passed"] for item in assertions) and all(item["passed"] for item in commands)
    return {"schema": 1, "timestamp": timestamp, "device": {"port": port, "baud": baud},
            "firmware_identity": identity_match.groupdict() if identity_match else None,
            "commands": commands, "assertions": assertions, "passed": passed}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="USART1 COM port, e.g. COM6 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--json-out", type=Path, required=True)
    parser.add_argument("--transcript-out", type=Path)
    args = parser.parse_args()

    if serial is None:
        print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
        return 2

    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        boot_log = read_until_idle(port, args.timeout)
        transcript = ["=== boot ===", boot_log]
        command_results = []
        for command in COMMANDS:
            response = send_command(port, command, args.timeout)
            transcript.extend([f"=== {command} ===", response])
            command_results.append({"command": command, "response": response})

    text = "\n".join(transcript)
    print(text)

    report = build_report(args.port, args.baud, boot_log, command_results,
                          dt.datetime.now(dt.timezone.utc).isoformat())
    args.json_out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.transcript_out:
        args.transcript_out.write_text(text, encoding="utf-8")
    if not report["passed"]:
        print("HIL smoke failed; see structured report", file=sys.stderr)
        return 1

    print("HIL smoke passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
