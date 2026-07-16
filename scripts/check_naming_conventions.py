#!/usr/bin/env python3
"""Validate Beta5 source naming and final capability directory names."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAYERS = ("App", "Service", "BSP", "Algorithm", "Platform")
FINAL_SERVICE_DIRS = {
    "motion_control", "state_estimation", "power_management", "safety_management", "command_management",
    "teleoperation", "line_following", "communication", "parameter_management", "system_monitoring",
}
SOURCE_NAME = re.compile(r"^[a-z][a-z0-9_]*\.(?:c|h)$")
DIRECTORY_NAME = re.compile(r"^[a-z][a-z0-9_]*$")
PUBLIC_SERVICE_HEADER = re.compile(r"^[a-z][a-z0-9_]*(?:_service|_config|_status|_types)\.h$")


def analyze(root: Path, final: bool = True) -> list[str]:
    root = Path(root).resolve()
    errors: list[str] = []
    for layer in LAYERS:
        base = root / layer
        if not base.exists():
            continue
        for path in base.rglob("*"):
            if path.is_dir():
                if path != base and not DIRECTORY_NAME.fullmatch(path.name):
                    errors.append(f"{path.relative_to(root).as_posix()}: directory name must be lower_snake_case")
                continue
            if path.suffix.lower() in {".c", ".h"} and not SOURCE_NAME.fullmatch(path.name):
                errors.append(f"{path.relative_to(root).as_posix()}: source name must be lower_snake_case")

    if (root / "Domain").exists():
        errors.append("Domain/: legacy layer name is forbidden in Beta5 final")
    service_root = root / "Service"
    actual = {path.name for path in service_root.iterdir() if path.is_dir()} if service_root.is_dir() else set()
    for name in sorted(actual - FINAL_SERVICE_DIRS):
        errors.append(f"Service/{name}: non-capability directory is forbidden in Beta5 final")
    for name in sorted(FINAL_SERVICE_DIRS - actual):
        errors.append(f"Service/{name}: required capability directory is missing")
    for name in sorted(actual & FINAL_SERVICE_DIRS):
        for header in (service_root / name).glob("*.h"):
            if not PUBLIC_SERVICE_HEADER.fullmatch(header.name):
                relative = header.relative_to(root).as_posix()
                errors.append(f"{relative}: final public Service header needs a contract suffix")
    return sorted(set(errors))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--final", action="store_true")
    return parser.parse_args()


def main(final: bool | None = None) -> int:
    if final is None:
        final = parse_args().final
    errors = analyze(ROOT, final=final)
    if errors:
        print("Naming convention check failed:")
        for error in errors:
            print(f"  - {error}")
        return 1
    print("Naming convention check passed (final).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
