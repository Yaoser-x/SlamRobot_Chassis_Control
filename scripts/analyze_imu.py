#!/usr/bin/env python3
"""Analyze IMU output stability and gyro-rate overlapping Allan deviation."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


ALGORITHM_VERSION = "gyro-oadev-v1"
AXES = ("x", "y", "z")
GYRO_COLUMNS = {axis: f"imu_gyro_corr_{axis}_dps" for axis in AXES}
DEFAULT_QUALITY_REJECT_MASK = 0x1FF
MIN_SEGMENT_SAMPLES = 64
MIN_OVERLAPPING_TERMS = 20
FIT_WINDOW_POINTS = 5


def correlation(a: list[float], b: list[float]) -> float | None:
    if len(a) != len(b) or len(a) < 2:
        return None
    am, bm = statistics.fmean(a), statistics.fmean(b)
    numerator = sum((x - am) * (y - bm) for x, y in zip(a, b))
    denominator = math.sqrt(sum((x - am) ** 2 for x in a) * sum((y - bm) ** 2 for y in b))
    return numerator / denominator if denominator else None


def linear_regression(x: list[float], y: list[float]) -> dict[str, float | None]:
    if len(x) != len(y) or len(x) < 2:
        return {"slope": None, "intercept": None, "r_squared": None}
    xm, ym = statistics.fmean(x), statistics.fmean(y)
    denominator = sum((value - xm) ** 2 for value in x)
    if denominator == 0:
        return {"slope": None, "intercept": None, "r_squared": None}
    slope = sum((a - xm) * (b - ym) for a, b in zip(x, y)) / denominator
    intercept = ym - slope * xm
    residual = sum((value - (slope * point + intercept)) ** 2 for point, value in zip(x, y))
    total = sum((value - ym) ** 2 for value in y)
    r_squared = 1.0 if total == 0.0 and residual == 0.0 else (1.0 - residual / total if total else 0.0)
    return {"slope": slope, "intercept": intercept, "r_squared": r_squared}


def _finite(row: dict[str, str], column: str) -> float:
    value = float(row[column])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {column}")
    return value


def _timestamp_ms(row: dict[str, str]) -> int:
    value = int(row["t_ms"], 0)
    if value < 0 or value > 0xFFFFFFFF:
        raise ValueError("t_ms must be a uint32 value")
    return value


def _elapsed_ms(previous_ms: int, current_ms: int) -> int:
    return (current_ms - previous_ms) & 0xFFFFFFFF


def _percentile(values: list[float], percentile: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return 0.0
    position = (len(ordered) - 1) * percentile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _quality_flags(row: dict[str, str]) -> int:
    text = row.get("imu_quality", row.get("quality_flags", "0")).strip()
    return int(text, 0) if text else 0


def _temperature(row: dict[str, str]) -> float | None:
    for column in ("imu_temp_c", "temperature_c"):
        if column in row and row[column].strip():
            value = float(row[column])
            return value if math.isfinite(value) else None
    return None


def _valid_sample(row: dict[str, str], quality_reject_mask: int) -> dict[str, object] | None:
    try:
        if (_quality_flags(row) & quality_reject_mask) != 0:
            return None
        return {
            "timestamp_ms": _timestamp_ms(row),
            "gyro": {axis: _finite(row, GYRO_COLUMNS[axis]) for axis in AXES},
            "temperature_c": _temperature(row),
        }
    except (KeyError, ValueError):
        return None


def _split_valid_segments(
    rows: list[dict[str, str]], quality_reject_mask: int
) -> tuple[list[list[dict[str, object]]], dict[str, object]]:
    parsed: list[dict[str, object] | None] = []
    quality_reject_count = 0
    invalid_sample_count = 0
    adjacent_dt_ms: list[int] = []
    previous: dict[str, object] | None = None

    for row in rows:
        quality_rejected = False
        try:
            quality_rejected = (_quality_flags(row) & quality_reject_mask) != 0
        except ValueError:
            quality_rejected = True
        sample = _valid_sample(row, quality_reject_mask)
        parsed.append(sample)
        if sample is None:
            if quality_rejected:
                quality_reject_count += 1
            else:
                invalid_sample_count += 1
            previous = None
            continue
        if previous is not None:
            elapsed = _elapsed_ms(int(previous["timestamp_ms"]), int(sample["timestamp_ms"]))
            if elapsed == 0 or elapsed >= 0x80000000:
                raise ValueError("timestamps must advance monotonically with uint32 wraparound")
            adjacent_dt_ms.append(elapsed)
        previous = sample

    if not adjacent_dt_ms:
        raise ValueError("at least two adjacent valid gyro samples are required")
    provisional_median_ms = statistics.median(adjacent_dt_ms)
    gap_threshold_ms = provisional_median_ms * 1.5
    segments: list[list[dict[str, object]]] = []
    current: list[dict[str, object]] = []
    gap_count = 0
    for sample in parsed:
        if sample is None:
            if current:
                segments.append(current)
                current = []
            continue
        if current:
            elapsed = _elapsed_ms(int(current[-1]["timestamp_ms"]), int(sample["timestamp_ms"]))
            if elapsed > gap_threshold_ms:
                segments.append(current)
                current = []
                gap_count += 1
        current.append(sample)
    if current:
        segments.append(current)

    usable = [segment for segment in segments if len(segment) >= MIN_SEGMENT_SAMPLES]
    short_segment_reject_count = sum(len(segment) for segment in segments if len(segment) < MIN_SEGMENT_SAMPLES)
    if not usable:
        raise ValueError(f"no continuous segment contains at least {MIN_SEGMENT_SAMPLES} samples")

    dt_ms = [
        _elapsed_ms(int(segment[index - 1]["timestamp_ms"]), int(segment[index]["timestamp_ms"]))
        for segment in usable
        for index in range(1, len(segment))
    ]
    median_dt_ms = statistics.median(dt_ms)
    jitter = [abs(value - median_dt_ms) / median_dt_ms for value in dt_ms]
    jitter_p95 = _percentile(jitter, 0.95)
    jitter_max = max(jitter, default=0.0)
    if jitter_p95 > 0.05 or jitter_max > 0.20:
        raise ValueError(
            f"sample jitter exceeds limit: p95={jitter_p95:.6f}, max={jitter_max:.6f}"
        )

    return usable, {
        "quality_reject_count": quality_reject_count,
        "invalid_sample_count": invalid_sample_count,
        "short_segment_reject_count": short_segment_reject_count,
        "gap_count": gap_count,
        "median_dt_s": median_dt_ms / 1000.0,
        "jitter_p95_ratio": jitter_p95,
        "jitter_max_ratio": jitter_max,
    }


def _axis_values(segment: list[dict[str, object]], axis: str, detrend_policy: str) -> list[float]:
    values = [float(sample["gyro"][axis]) for sample in segment]  # type: ignore[index]
    if detrend_policy == "none":
        return values
    times = [
        _elapsed_ms(int(segment[0]["timestamp_ms"]), int(sample["timestamp_ms"])) / 1000.0
        for sample in segment
    ]
    fit = linear_regression(times, values)
    slope = float(fit["slope"] or 0.0)
    intercept = float(fit["intercept"] or 0.0)
    return [value - (slope * time_s + intercept) for time_s, value in zip(times, values)]


def overlapping_allan_deviation(
    segments: list[list[dict[str, object]]], sample_period_s: float, detrend_policy: str
) -> list[dict[str, float | int]]:
    prepared = [
        {axis: _axis_values(segment, axis, detrend_policy) for axis in AXES}
        for segment in segments
    ]
    curves: list[dict[str, float | int]] = []
    cluster = 1
    while True:
        squared = {axis: 0.0 for axis in AXES}
        term_count = 0
        for values_by_axis in prepared:
            length = len(values_by_axis["x"])
            terms = length - 2 * cluster + 1
            if terms <= 0:
                continue
            prefixes = {
                axis: [0.0] for axis in AXES
            }
            for axis in AXES:
                for value in values_by_axis[axis]:
                    prefixes[axis].append(prefixes[axis][-1] + value)
            for index in range(terms):
                for axis in AXES:
                    first = (prefixes[axis][index + cluster] - prefixes[axis][index]) / cluster
                    second = (
                        prefixes[axis][index + 2 * cluster] - prefixes[axis][index + cluster]
                    ) / cluster
                    squared[axis] += (second - first) ** 2
            term_count += terms
        if term_count < MIN_OVERLAPPING_TERMS:
            break
        point: dict[str, float | int] = {
            "tau_s": cluster * sample_period_s,
            "overlapping_terms": term_count,
        }
        for axis in AXES:
            point[f"adev_{axis}_dps"] = math.sqrt(squared[axis] / (2.0 * term_count))
        curves.append(point)
        cluster *= 2
    return curves


def _fit_windows(
    curve: list[dict[str, float | int]], axis: str, expected_slope: float
) -> list[dict[str, float]]:
    candidates: list[dict[str, float]] = []
    key = f"adev_{axis}_dps"
    for start in range(0, len(curve) - FIT_WINDOW_POINTS + 1):
        window = curve[start:start + FIT_WINDOW_POINTS]
        if any(float(point[key]) <= 0.0 for point in window):
            continue
        x = [math.log10(float(point["tau_s"])) for point in window]
        y = [math.log10(float(point[key])) for point in window]
        fit = linear_regression(x, y)
        slope = fit["slope"]
        r_squared = fit["r_squared"]
        if slope is None or r_squared is None:
            continue
        if abs(slope - expected_slope) <= 0.1 and r_squared >= 0.90:
            center_log_tau = statistics.fmean(x)
            representative = 10.0 ** (float(fit["intercept"]) + slope * center_log_tau)
            candidates.append({
                "slope": slope,
                "r_squared": r_squared,
                "tau_center_s": 10.0 ** center_log_tau,
                "representative_adev_dps": representative,
                "score": r_squared - abs(slope - expected_slope),
            })
    return candidates


def _best_fit(curve: list[dict[str, float | int]], axis: str, expected_slope: float) -> dict[str, float] | None:
    candidates = _fit_windows(curve, axis, expected_slope)
    return max(candidates, key=lambda candidate: candidate["score"]) if candidates else None


def extract_gyro_coefficients(
    curve: list[dict[str, float | int]], axis: str
) -> dict[str, dict[str, float] | None]:
    arw_fit = _best_fit(curve, axis, -0.5)
    bi_fit = _best_fit(curve, axis, 0.0)
    rrw_fit = _best_fit(curve, axis, 0.5)

    arw = None
    if arw_fit is not None:
        coefficient_deg_sqrt_s = (
            arw_fit["representative_adev_dps"] * math.sqrt(arw_fit["tau_center_s"])
        )
        arw = {
            **arw_fit,
            "deg_per_sqrt_hour": coefficient_deg_sqrt_s * 60.0,
            "rad_per_sqrt_second": coefficient_deg_sqrt_s * math.pi / 180.0,
        }

    bias_instability = None
    if bi_fit is not None:
        bias_dps = bi_fit["representative_adev_dps"] / 0.664
        bias_instability = {**bi_fit, "deg_per_hour": bias_dps * 3600.0}

    rate_random_walk = None
    if rrw_fit is not None:
        coefficient_deg_per_s_sqrt_s = (
            rrw_fit["representative_adev_dps"] / math.sqrt(rrw_fit["tau_center_s"])
        )
        rate_random_walk = {
            **rrw_fit,
            "deg_per_s_per_sqrt_second": coefficient_deg_per_s_sqrt_s,
            "deg_per_hour_per_sqrt_hour": coefficient_deg_per_s_sqrt_s * 216000.0,
        }
    return {
        "angle_random_walk": arw,
        "bias_instability": bias_instability,
        "rate_random_walk": rate_random_walk,
    }


def _angular_drift_deg_per_hour(segments: list[list[dict[str, object]]]) -> dict[str, float]:
    result = {}
    for axis in AXES:
        duration_weighted_rate = 0.0
        total_duration_s = 0.0
        for segment in segments:
            duration_s = _elapsed_ms(
                int(segment[0]["timestamp_ms"]), int(segment[-1]["timestamp_ms"])
            ) / 1000.0
            if duration_s <= 0.0:
                continue
            mean_rate = statistics.fmean(float(sample["gyro"][axis]) for sample in segment)  # type: ignore[index]
            duration_weighted_rate += mean_rate * duration_s
            total_duration_s += duration_s
        result[axis] = duration_weighted_rate / total_duration_s * 3600.0 if total_duration_s else 0.0
    return result


def analyze_gyro(
    rows: list[dict[str, str]],
    metadata: dict[str, object] | None = None,
    detrend_policy: str = "none",
    quality_reject_mask: int = DEFAULT_QUALITY_REJECT_MASK,
) -> tuple[dict[str, object], list[dict[str, float | int]]]:
    if detrend_policy not in ("none", "linear"):
        raise ValueError("detrend_policy must be none or linear")
    segments, sampling = _split_valid_segments(rows, quality_reject_mask)
    curve = overlapping_allan_deviation(segments, float(sampling["median_dt_s"]), detrend_policy)
    if len(curve) < FIT_WINDOW_POINTS:
        raise ValueError("not enough OADEV points for five-point coefficient fitting")
    temperatures = [
        float(sample["temperature_c"])
        for segment in segments
        for sample in segment
        if sample["temperature_c"] is not None
    ]
    details = {
        "algorithm_version": ALGORITHM_VERSION,
        "estimator": "overlapping_allan_deviation",
        "input_unit": "deg/s",
        "quality_reject_mask": f"0x{quality_reject_mask:08X}",
        "sample_count": len(rows),
        "accepted_sample_count": sum(len(segment) for segment in segments),
        "valid_segments": [len(segment) for segment in segments],
        "sample_rate_hz": 1.0 / float(sampling["median_dt_s"]),
        "temperature_min_c": min(temperatures) if temperatures else None,
        "temperature_max_c": max(temperatures) if temperatures else None,
        "linear_drift_deg_per_hour": _angular_drift_deg_per_hour(segments),
        "detrend_policy": detrend_policy,
        **sampling,
    }
    if metadata:
        details.update(metadata)
    details["axes"] = {axis: extract_gyro_coefficients(curve, axis) for axis in AXES}
    details["curve"] = curve
    return details, curve


def analyze(rows: list[dict[str, str]]) -> dict[str, object]:
    """Keep the original pose analysis separate from gyro-rate OADEV conclusions."""
    yaw = [float(row["yaw_deg"]) for row in rows]
    roll = [float(row["roll_deg"]) for row in rows]
    pitch = [float(row["pitch_deg"]) for row in rows]
    temperature = [float(row["temperature_c"]) for row in rows]
    time_s = [float(row["t_ms"]) / 1000.0 for row in rows]
    duration = time_s[-1] - time_s[0] if len(time_s) > 1 else 0.0
    result = {
        "samples": len(rows),
        "stationary_stddev_deg": {
            "roll": statistics.pstdev(roll),
            "pitch": statistics.pstdev(pitch),
            "yaw": statistics.pstdev(yaw),
        },
        "peak_to_peak_deg": {
            "roll": max(roll) - min(roll),
            "pitch": max(pitch) - min(pitch),
            "yaw": max(yaw) - min(yaw),
        },
        "yaw_output_stability": {
            "drift_deg_per_min": ((yaw[-1] - yaw[0]) / duration * 60.0) if duration else None,
            "temperature_correlation": correlation(temperature, yaw),
            "temperature_regression": linear_regression(temperature, yaw),
        },
        "level_return_error_deg": math.hypot(roll[-1], pitch[-1]),
    }
    faces = {}
    if rows and "face" in rows[0]:
        for face in sorted({row["face"] for row in rows}):
            group = [row for row in rows if row["face"] == face]
            faces[face] = {
                axis: statistics.fmean(float(row[axis]) for row in group)
                for axis in ("accel_x_g", "accel_y_g", "accel_z_g")
            }
    result["six_face_report"] = faces
    return result


def write_artifacts(
    output_dir: Path, result: dict[str, object], curve: list[dict[str, float | int]]
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "allan_result.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    with (output_dir / "allan_curve.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(curve[0]))
        writer.writeheader()
        writer.writerows(curve)
    lines = [
        "# Gyro overlapping Allan deviation",
        "",
        f"- Algorithm: `{result['algorithm_version']}`",
        f"- Firmware SHA: `{result.get('firmware_sha', 'unknown')}`",
        f"- Parameter CRC: `{result.get('parameter_crc', 'unknown')}`",
        f"- Samples: {result['accepted_sample_count']} accepted / {result['sample_count']} input",
        f"- Valid segments: {result['valid_segments']}",
        f"- Sample rate: {float(result['sample_rate_hz']):.6f} Hz",
        f"- Detrend policy: `{result['detrend_policy']}`",
        "",
        "| Axis | ARW (deg/sqrt(h)) | BI (deg/h) | RRW (deg/h/sqrt(h)) |",
        "|---|---:|---:|---:|",
    ]
    axes = result["axes"]
    for axis in AXES:
        coefficients = axes[axis]  # type: ignore[index]
        arw = coefficients["angle_random_walk"]
        bi = coefficients["bias_instability"]
        rrw = coefficients["rate_random_walk"]
        arw_text = f"{arw['deg_per_sqrt_hour']:.9g}" if arw else "null"
        bi_text = f"{bi['deg_per_hour']:.9g}" if bi else "null"
        rrw_text = f"{rrw['deg_per_hour_per_sqrt_hour']:.9g}" if rrw else "null"
        lines.append(f"| {axis.upper()} | {arw_text} | {bi_text} | {rrw_text} |")
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--firmware-sha", required=True)
    parser.add_argument("--parameter-crc", required=True)
    parser.add_argument("--imu-odr", required=True)
    parser.add_argument("--imu-bandwidth-profile", required=True)
    parser.add_argument("--input-signal-stage", default="corrected_unfiltered")
    parser.add_argument("--bias-correction-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--filter-enabled", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--detrend", choices=("none", "linear"), default="none")
    parser.add_argument("--quality-reject-mask", type=lambda value: int(value, 0), default=DEFAULT_QUALITY_REJECT_MASK)
    args = parser.parse_args()
    with args.csv.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    metadata = {
        "firmware_sha": args.firmware_sha,
        "parameter_crc": args.parameter_crc,
        "input_signal_stage": args.input_signal_stage,
        "imu_odr": args.imu_odr,
        "imu_bandwidth_profile": args.imu_bandwidth_profile,
        "bias_correction_enabled": args.bias_correction_enabled,
        "filter_enabled": args.filter_enabled,
    }
    result, curve = analyze_gyro(rows, metadata, args.detrend, args.quality_reject_mask)
    write_artifacts(args.output_dir, result, curve)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
