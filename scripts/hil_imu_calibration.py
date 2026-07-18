#!/usr/bin/env python3
"""Verify that a stationary IMU calibration does not add RTOS task timeouts."""

from __future__ import annotations

import argparse
import re
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover - user environment check
    serial = None

from hil_smoke import read_until_idle, send_command


MONITORED_TASKS = ("line", "esp", "debug", "led", "oled")
RTOS_LINE = re.compile(
    r"^RTOS\s+(?P<task>\w+)\s+.*?timeout=(?P<timeout>\d+)",
    re.MULTILINE,
)
IMU_CALIBRATION_SUCCESS = re.compile(r"acal=4,\d+,1")


def timeout_snapshot(text: str) -> dict[str, int]:
    values = {
        match.group("task"): int(match.group("timeout"))
        for match in RTOS_LINE.finditer(text)
    }
    missing = [task for task in MONITORED_TASKS if task not in values]
    if missing:
        raise AssertionError(f"missing RTOS task lines: {', '.join(missing)}")
    return {task: values[task] for task in MONITORED_TASKS}


def calibration_succeeded(text: str) -> bool:
    return IMU_CALIBRATION_SUCCESS.search(text) is not None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--calibration-timeout", type=float, default=9.0)
    args = parser.parse_args()

    if serial is None:
        print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
        return 2

    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        read_until_idle(port, args.timeout)
        before_text = send_command(port, "rtos", args.timeout)
        before = timeout_snapshot(before_text)
        response = send_command(port, "imucal 500", args.timeout)
        if "calibration accepted" not in response:
            raise AssertionError(f"IMU calibration was not accepted:\n{response}")

        # Keep the measurement window quiet. Repeated full `status` responses
        # can occupy the debug task long enough to starve low-priority UI tasks
        # and create the very timeout increments this check is measuring.
        time.sleep(args.calibration_timeout)
        after_text = send_command(port, "rtos", args.timeout)
        after = timeout_snapshot(after_text)

        status_text = send_command(port, "status", args.timeout)
        if not calibration_succeeded(status_text):
            raise AssertionError("IMU calibration did not finish before timeout")

    regressions = {
        task: (before[task], after[task])
        for task in MONITORED_TASKS
        if after[task] != before[task]
    }
    if regressions:
        raise AssertionError(f"task timeout counters changed: {regressions}")
    print(f"IMU calibration scheduling passed: {after}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
