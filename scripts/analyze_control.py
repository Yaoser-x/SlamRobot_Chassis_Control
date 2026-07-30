#!/usr/bin/env python3
"""Compute offline chassis-control metrics from CSV evidence without changing parameters."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def _finite(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def analyze(rows: list[dict[str, str]], saturation_permille: float = 1000.0) -> dict[str, float | None]:
    if len(rows) < 2 or saturation_permille <= 0.0:
        raise ValueError("at least two rows and a positive saturation limit are required")
    samples = [{name: _finite(row, name) for name in (
        "time_s", "target_mps", "left_mps", "right_mps", "left_permille", "right_permille", "heading_error_deg"
    )} for row in rows]
    if any(samples[index]["time_s"] <= samples[index - 1]["time_s"] for index in range(1, len(samples))):
        raise ValueError("time_s must be strictly increasing")
    measured = [(sample["left_mps"] + sample["right_mps"]) * 0.5 for sample in samples]
    target = samples[-1]["target_mps"]
    amplitude = target - measured[0]
    rise_time = None
    if abs(amplitude) > 1.0e-9:
        low = measured[0] + 0.1 * amplitude
        high = measured[0] + 0.9 * amplitude
        crossed_low = next((sample["time_s"] for sample, value in zip(samples, measured)
                            if (value - low) * amplitude >= 0.0), None)
        crossed_high = next((sample["time_s"] for sample, value in zip(samples, measured)
                             if (value - high) * amplitude >= 0.0), None)
        if crossed_low is not None and crossed_high is not None:
            rise_time = crossed_high - crossed_low
    tail = max(1, len(samples) // 10)
    steady_error = sum(target - value for value in measured[-tail:]) / tail
    peak = max(measured) if amplitude >= 0.0 else min(measured)
    overshoot = max(0.0, (peak - target) * (1.0 if amplitude >= 0.0 else -1.0))
    saturated = sum(1 for sample in samples if max(abs(sample["left_permille"]), abs(sample["right_permille"]))
                    >= saturation_permille)
    return {
        "rise_time_s": rise_time,
        "steady_state_error_mps": steady_error,
        "overshoot_mps": overshoot,
        "saturation_ratio": saturated / len(samples),
        "mean_left_right_speed_difference_mps": sum(
            abs(sample["left_mps"] - sample["right_mps"]) for sample in samples
        ) / len(samples),
        "mean_absolute_heading_error_deg": sum(abs(sample["heading_error_deg"]) for sample in samples) / len(samples),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--saturation-permille", type=float, default=1000.0)
    args = parser.parse_args()
    with args.csv_path.open(newline="", encoding="utf-8") as source:
        report = analyze(list(csv.DictReader(source)), args.saturation_permille)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
