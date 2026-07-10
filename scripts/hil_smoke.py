#!/usr/bin/env python3
"""Run a read-only HIL smoke test through the USART1 debug console."""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover - user environment check
    serial = None


COMMANDS = ("status", "i2cscan", "imutest", "espflash status")


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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="USART1 COM port, e.g. COM6 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    if serial is None:
        print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
        return 2

    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        boot_log = read_until_idle(port, args.timeout)
        transcript = ["=== boot ===", boot_log]
        for command in COMMANDS:
            response = send_command(port, command, args.timeout)
            transcript.extend([f"=== {command} ===", response])

    text = "\n".join(transcript)
    print(text)

    required = ("POST:", "POST done=", "PARAM ")
    missing = [needle for needle in required if needle not in text]
    if missing:
        print(f"HIL smoke failed, missing: {', '.join(missing)}", file=sys.stderr)
        return 1
    if "bmi270 probe failed" in text:
        print("HIL smoke failed: BMI270 probe failed", file=sys.stderr)
        return 1

    print("HIL smoke passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
