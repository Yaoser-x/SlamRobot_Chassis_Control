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


def linear_regression(x: list[float], y: list[float]) -> dict[str, float | None]:
    if len(x) != len(y) or len(x) < 2:
        return {"slope": None, "intercept": None}
    xm, ym = statistics.fmean(x), statistics.fmean(y)
    denominator = sum((value - xm) ** 2 for value in x)
    if denominator == 0:
        return {"slope": None, "intercept": None}
    slope = sum((a - xm) * (b - ym) for a, b in zip(x, y)) / denominator
    return {"slope": slope, "intercept": ym - slope * xm}


def allan_deviation(values: list[float], sample_period_s: float) -> list[dict[str, float]]:
    result = []
    cluster = 1
    while cluster * 2 <= len(values):
        means = [statistics.fmean(values[i:i + cluster]) for i in range(0, len(values) - cluster + 1, cluster)]
        if len(means) >= 2:
            variance = sum((means[i + 1] - means[i]) ** 2 for i in range(len(means) - 1)) / (2 * (len(means) - 1))
            result.append({"tau_s": cluster * sample_period_s, "deviation": math.sqrt(variance)})
        cluster *= 2
    return result


def analyze(rows: list[dict[str, str]]) -> dict[str, object]:
    yaw = [float(row["yaw_deg"]) for row in rows]
    roll = [float(row["roll_deg"]) for row in rows]
    pitch = [float(row["pitch_deg"]) for row in rows]
    temperature = [float(row["temperature_c"]) for row in rows]
    time_s = [float(row["t_ms"]) / 1000.0 for row in rows]
    duration = time_s[-1] - time_s[0] if len(time_s) > 1 else 0.0
    result = {
        "samples": len(rows),
        "stationary_stddev_deg": {"roll": statistics.pstdev(roll), "pitch": statistics.pstdev(pitch),
                                  "yaw": statistics.pstdev(yaw)},
        "peak_to_peak_deg": {"roll": max(roll) - min(roll), "pitch": max(pitch) - min(pitch),
                             "yaw": max(yaw) - min(yaw)},
        "yaw_drift_deg_per_min": ((yaw[-1] - yaw[0]) / duration * 60.0) if duration else None,
        "level_return_error_deg": math.hypot(roll[-1], pitch[-1]),
        "temperature_yaw_correlation": correlation(temperature, yaw),
        "temperature_yaw_regression": linear_regression(temperature, yaw),
    }
    if len(time_s) > 1:
        sample_period_s = statistics.median(b - a for a, b in zip(time_s, time_s[1:]))
        result["allan_deviation_yaw_deg"] = allan_deviation(yaw, sample_period_s)
    faces = {}
    if rows and "face" in rows[0]:
        for face in sorted({row["face"] for row in rows}):
            group = [row for row in rows if row["face"] == face]
            faces[face] = {axis: statistics.fmean(float(row[axis]) for row in group)
                           for axis in ("accel_x_g", "accel_y_g", "accel_z_g")}
    result["six_face_report"] = faces
    return result


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
