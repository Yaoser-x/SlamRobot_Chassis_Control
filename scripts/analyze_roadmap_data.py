#!/usr/bin/env python3
"""Analyze Roadmap calibration/HIL CSV data using only the Python standard library."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def _floats(rows: list[dict[str, str]], name: str) -> list[float]:
    return [float(row[name]) for row in rows if row.get(name, "") != ""]


def _summary(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "min": min(values),
        "max": max(values),
        "peak_to_peak": max(values) - min(values),
        "stdev": statistics.pstdev(values),
    }


def analyze_current(rows: list[dict[str, str]]) -> dict[str, object]:
    motors = {}
    for motor in range(1, 5):
        name = f"m{motor}_current_a"
        if rows and name in rows[0]:
            motors[f"m{motor}"] = _summary(_floats(rows, name))
    return {"kind": "current", "motors": motors}


def analyze_encoder(rows: list[dict[str, str]]) -> dict[str, object]:
    motors = {}
    for motor in range(1, 5):
        speed_name = f"m{motor}_speed_mps"
        valid_name = f"m{motor}_valid"
        if rows and speed_name in rows[0]:
            speeds = _floats(rows, speed_name)
            valid = _floats(rows, valid_name) if valid_name in rows[0] else []
            motors[f"m{motor}"] = {
                "speed": _summary(speeds),
                "valid_ratio": statistics.fmean(valid) if valid else None,
            }
    return {"kind": "encoder", "motors": motors}


def analyze_line(rows: list[dict[str, str]]) -> dict[str, object]:
    channels = {}
    accepted = True
    for channel in range(8):
        name = f"ch{channel}"
        floor = [float(row[name]) for row in rows if row.get("surface") == "floor"]
        line = [float(row[name]) for row in rows if row.get("surface") == "line"]
        separation = abs(statistics.fmean(floor) - statistics.fmean(line)) if floor and line else 0.0
        channel_ok = separation >= 50.0
        accepted &= channel_ok
        channels[name] = {
            "floor": _summary(floor), "line": _summary(line),
            "threshold": (statistics.fmean(floor) + statistics.fmean(line)) / 2.0 if floor and line else None,
            "active_low": statistics.fmean(line) < statistics.fmean(floor) if floor and line else None,
            "separation": separation, "accepted": channel_ok,
        }
    return {"kind": "line", "accepted": accepted, "channels": channels}


def analyze_geometry(rows: list[dict[str, str]]) -> dict[str, object]:
    distance = _floats(rows, "distance_m")
    lateral = _floats(rows, "lateral_error_m")
    yaw = _floats(rows, "yaw_error_deg")
    return {"kind": "geometry", "distance_m": _summary(distance),
            "lateral_error_m": _summary(lateral), "yaw_error_deg": _summary(yaw)}


ANALYZERS = {"current": analyze_current, "encoder": analyze_encoder,
             "line": analyze_line, "geometry": analyze_geometry}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=ANALYZERS)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--json", type=Path)
    parser.add_argument("--markdown", type=Path)
    args = parser.parse_args()
    with args.csv.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    result = ANALYZERS[args.kind](rows)
    text = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    if args.json:
        args.json.write_text(text + "\n", encoding="utf-8")
    if args.markdown:
        args.markdown.write_text(f"# {args.kind.title()} analysis\n\n```json\n{text}\n```\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
