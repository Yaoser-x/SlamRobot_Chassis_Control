#!/usr/bin/env python3
"""Report heuristic source-complexity signals for the five firmware layers."""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAYERS = ("App", "Service", "Algorithm", "Domain", "BSP", "Platform")
SOURCE_SUFFIXES = {".c", ".h"}
INCLUDE = re.compile(r"^\s*#\s*include\b", re.MULTILINE)
STATIC_FUNCTION = re.compile(
    r"^\s*static\s+(?!const\b)(?:[A-Za-z_]\w*[\s*]+)+[A-Za-z_]\w*\s*\([^;]*\)\s*\{",
    re.MULTILINE,
)
HAL_CALL = re.compile(r"\bHAL_[A-Za-z0-9_]+\s*\(")
RTOS_CALL = re.compile(r"\b(?:os[A-Z][A-Za-z0-9_]*|xTask[A-Za-z0-9_]*|vTask[A-Za-z0-9_]*)\s*\(")
GLOBAL_DECLARATION = re.compile(
    r"^(?!\s)(?!#)(?!typedef\b)(?!extern\b)(?!static\s+(?:inline\s+)?[^;{]+\([^;]*\))"
    r"(?:static\s+)?(?:const\s+)?[A-Za-z_]\w*(?:\s+[A-Za-z_]\w*)*[\s*]+[A-Za-z_]\w*(?:\[[^]]*\])?\s*(?:=[^;]*)?;",
    re.MULTILINE,
)


def source_files() -> list[Path]:
    return sorted(
        path
        for layer in LAYERS
        for path in (ROOT / layer).rglob("*")
        if path.suffix.lower() in SOURCE_SUFFIXES
    )


def main() -> int:
    print("lines\tstatic_functions\tincludes\tmodule_globals\thal_calls\trtos_calls\tfile")
    for path in source_files():
        text = path.read_text(encoding="utf-8", errors="replace")
        print(
            f"{len(text.splitlines())}\t{len(STATIC_FUNCTION.findall(text))}\t"
            f"{len(INCLUDE.findall(text))}\t{len(GLOBAL_DECLARATION.findall(text))}\t"
            f"{len(HAL_CALL.findall(text))}\t{len(RTOS_CALL.findall(text))}\t"
            f"{path.relative_to(ROOT).as_posix()}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
