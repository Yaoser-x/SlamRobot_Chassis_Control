#!/usr/bin/env python3
"""Analyze supervised bidirectional straight-line HIL data without controlling the vehicle."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any


TRANSITION_DISTANCE_M = 0.30
TRIM_LIMIT_MPS = 0.075
LATERAL_LIMIT_M = 0.05
YAW_LIMIT_DEG = 2.0
REQUIRED_MATRIX = {"forward_015", "forward_030", "reverse_015", "reverse_030"}


def _truthy(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def _group_key(direction: str, speed_mps: float) -> str:
    normalized_direction = direction.strip().lower()
    if normalized_direction not in {"forward", "reverse"}:
        raise ValueError(f"unsupported direction: {direction}")
    if abs(speed_mps - 0.15) <= 0.002:
        speed_code = "015"
    elif abs(speed_mps - 0.30) <= 0.002:
        speed_code = "030"
    else:
        raise ValueError(f"unsupported straight speed: {speed_mps}")
    return f"{normalized_direction}_{speed_code}"


def _trim_name(group: str) -> str:
    return f"straight_trim_{group}_mps"


def _median_abs(rows: list[dict[str, str]], field: str) -> float | None:
    values = [abs(float(row[field])) for row in rows if row.get(field, "") != ""]
    return statistics.median(values) if values else None


def _matrix(measurements: list[dict[str, str]]) -> dict[str, dict[str, Any]]:
    groups: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in measurements:
        groups[_group_key(row["direction"], float(row["speed_mps"]))].append(row)

    result: dict[str, dict[str, Any]] = {}
    for key, rows in sorted(groups.items()):
        transition = [row for row in rows if _truthy(row.get("caster_transition", "0"))]
        steady = [row for row in rows if not _truthy(row.get("caster_transition", "0"))]
        transition_run_ids = {row["run_id"] for row in transition}
        steady_run_ids = {row["run_id"] for row in steady}
        lateral = _median_abs(steady, "lateral_error_m")
        yaw = _median_abs(steady, "yaw_error_deg")
        distance_ok = bool(steady) and all(float(row["distance_m"]) >= 2.0 for row in steady)
        result[key] = {
            "caster_transition_runs": len(transition_run_ids),
            "steady_runs": len(steady_run_ids),
            "median_lateral_error_m": lateral,
            "median_yaw_error_deg": yaw,
            "distance_ok": distance_ok,
            "meets_geometry": bool(distance_ok and lateral is not None and yaw is not None and
                                   lateral <= LATERAL_LIMIT_M and yaw <= YAW_LIMIT_DEG),
            "firmware_sha": sorted({row.get("firmware_sha", "") for row in rows if row.get("firmware_sha")}),
        }
    return result


def analyze(telemetry: list[dict[str, Any]],
            measurements: list[dict[str, str]],
            current_trims: dict[str, float],
            baseline_measurements: list[dict[str, str]] | None = None) -> dict[str, Any]:
    run_groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    run_metadata: dict[str, tuple[str, float]] = {}
    for row in measurements:
        run_metadata[row["run_id"]] = (row["direction"], float(row["speed_mps"]))
    for row in telemetry:
        if float(row.get("straight_transition_distance_m", 0.0)) >= TRANSITION_DISTANCE_M:
            run_id = row.get("run_id")
            if run_id is None:
                if len(run_metadata) != 1:
                    raise ValueError("telemetry run_id is required when measurements contain multiple runs")
                run_id = next(iter(run_metadata))
            run_groups[str(run_id)].append(row)

    wheel_deltas: dict[str, list[float]] = defaultdict(list)
    for run_id, rows in run_groups.items():
        if run_id not in run_metadata:
            continue
        group = _group_key(*run_metadata[run_id])
        wheel_deltas[group].extend(
            (float(row["actual_left_mps"]) - float(row["actual_right_mps"])) / 2.0
            for row in rows
        )

    trim_updates: dict[str, float] = {}
    for group, values in sorted(wheel_deltas.items()):
        name = _trim_name(group)
        proposed = float(current_trims.get(name, 0.0)) + statistics.median(values)
        trim_updates[name] = max(-TRIM_LIMIT_MPS, min(TRIM_LIMIT_MPS, proposed))

    matrix = _matrix(measurements)
    baseline = _matrix(baseline_measurements or [])
    improvements: dict[str, float | None] = {}
    for group, metrics in matrix.items():
        previous = baseline.get(group, {}).get("median_lateral_error_m")
        current = metrics["median_lateral_error_m"]
        improvements[group] = ((previous - current) / previous) if previous and current is not None else None

    matrix_complete = set(matrix) == REQUIRED_MATRIX
    enough_final_runs = matrix_complete and all(matrix[group]["steady_runs"] >= 5
                                                for group in REQUIRED_MATRIX)
    geometry_pass = matrix_complete and all(matrix[group]["meets_geometry"]
                                            for group in REQUIRED_MATRIX)
    improvement_pass = matrix_complete and all(improvements.get(group) is not None and
                                               improvements[group] >= 0.50
                                               for group in REQUIRED_MATRIX)
    firmware_values = [row.get("firmware_sha", "").strip() for row in measurements]
    baseline_values = [row.get("firmware_sha", "").strip()
                       for row in (baseline_measurements or [])]
    firmware_ids = set(firmware_values)
    baseline_ids = set(baseline_values)
    identity_complete = bool(firmware_values) and all(firmware_values)
    baseline_identity_complete = not baseline_measurements or all(baseline_values)
    firmware_identity_consistent = identity_complete and len(firmware_ids) == 1 and (
        not baseline_measurements or
        (baseline_identity_complete and len(baseline_ids) == 1 and baseline_ids == firmware_ids))
    return {
        "transition_distance_m": TRANSITION_DISTANCE_M,
        "matrix": matrix,
        "baseline_improvement_ratio": improvements,
        "trim_updates": trim_updates,
        "ram_commands": [f"set {name} {value:.6f}" for name, value in trim_updates.items()],
        "acceptance": {
            "matrix_complete": matrix_complete,
            "firmware_identity_consistent": firmware_identity_consistent,
            "enough_final_runs": enough_final_runs,
            "geometry_pass": geometry_pass,
            "improvement_pass": improvement_pass,
            "passed": bool(firmware_identity_consistent and enough_final_runs and
                           geometry_pass and improvement_pass),
        },
    }


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    with path.open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def _read_csv(path: Path | None) -> list[dict[str, str]]:
    if path is None:
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _markdown(report: dict[str, Any]) -> str:
    lines = ["# Bidirectional straight-line HIL", "", f"Overall pass: `{report['acceptance']['passed']}`", ""]
    for group, metrics in report["matrix"].items():
        lines.append(f"- {group}: steady={metrics['steady_runs']}, lateral={metrics['median_lateral_error_m']}, "
                     f"yaw={metrics['median_yaw_error_deg']}, geometry={metrics['meets_geometry']}")
    lines.extend(["", "## RAM-only commands", "", "```text",
                  *report["ram_commands"], "```", ""])
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("telemetry_jsonl", type=Path)
    parser.add_argument("measurements_csv", type=Path)
    parser.add_argument("--baseline-csv", type=Path)
    parser.add_argument("--current-trims", type=Path,
                        help="JSON object keyed by ParamStore trim field")
    parser.add_argument("--json", type=Path)
    parser.add_argument("--markdown", type=Path)
    args = parser.parse_args()
    current = json.loads(args.current_trims.read_text(encoding="utf-8")) if args.current_trims else {}
    report = analyze(_read_jsonl(args.telemetry_jsonl), _read_csv(args.measurements_csv),
                     current, _read_csv(args.baseline_csv))
    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    if args.json:
        args.json.write_text(text + "\n", encoding="utf-8")
    if args.markdown:
        args.markdown.write_text(_markdown(report), encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
