#!/usr/bin/env python3
"""Validate final unique runtime motor-output ownership."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCAN_ROOTS = ("Core", "App", "Service", "Platform")
SOURCE_SUFFIXES = {".c", ".h"}
MOTOR_OUTPUT_API = re.compile(
    r"\bMotorDriver_(?:Init|SetSpeedGetter|SetDirectionConfig|SetPermille|SetSidePermille|Stop|StopSide|StopAll)\s*\("
)
def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", " ", text)


def analyze(root: Path, final: bool = True) -> list[str]:
    root = Path(root).resolve()
    errors: list[str] = []
    for scan_root in SCAN_ROOTS:
        base = root / scan_root
        if not base.exists():
            continue
        for path in base.rglob("*"):
            if path.suffix.lower() not in SOURCE_SUFFIXES:
                continue
            relative = path.relative_to(root).as_posix()
            code = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
            for match in MOTOR_OUTPUT_API.finditer(code):
                final_owner = relative.startswith("Service/motion_control/")
                if not final_owner:
                    line = code.count("\n", 0, match.start()) + 1
                    errors.append(f"{relative}:{line}: runtime motor output API is owned only by Motion Control")
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
        print("Service ownership check failed:")
        for error in errors:
            print(f"  - {error}")
        return 1
    print("Service ownership check passed (final motor output owner).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
