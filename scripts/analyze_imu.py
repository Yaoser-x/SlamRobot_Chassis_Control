#!/usr/bin/env python3
"""Compute stationary noise, yaw drift, leveling error, and temperature correlation."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def correlation(a: list[float], b: list[float]) -> float | None:
    if len(a) != len(b) or len(a) < 2:
        return None
    am, bm = statistics.fmean(a), statistics.fmean(b)
    numerator = sum((x - am) * (y - bm) for x, y in zip(a, b))
    denominator = math.sqrt(sum((x - am) ** 2 for x in a) * sum((y - bm) ** 2 for y in b))
    return numerator / denominator if denominator else None


def analyze(rows: list[dict[str, str]]) -> dict[str, object]:
    yaw = [float(row["yaw_deg"]) for row in rows]
    roll = [float(row["roll_deg"]) for row in rows]
    pitch = [float(row["pitch_deg"]) for row in rows]
    temperature = [float(row["temperature_c"]) for row in rows]
    time_s = [float(row["t_ms"]) / 1000.0 for row in rows]
    duration = time_s[-1] - time_s[0] if len(time_s) > 1 else 0.0
    return {
        "samples": len(rows),
        "stationary_stddev_deg": {"roll": statistics.pstdev(roll), "pitch": statistics.pstdev(pitch),
                                  "yaw": statistics.pstdev(yaw)},
        "peak_to_peak_deg": {"roll": max(roll) - min(roll), "pitch": max(pitch) - min(pitch),
                             "yaw": max(yaw) - min(yaw)},
        "yaw_drift_deg_per_min": ((yaw[-1] - yaw[0]) / duration * 60.0) if duration else None,
        "level_return_error_deg": math.hypot(roll[-1], pitch[-1]),
        "temperature_yaw_correlation": correlation(temperature, yaw),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    with args.csv.open(newline="", encoding="utf-8") as handle:
        result = analyze(list(csv.DictReader(handle)))
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.json:
        args.json.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
