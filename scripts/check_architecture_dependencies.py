#!/usr/bin/env python3
"""Validate five-layer include direction and hardware API boundaries."""

from __future__ import annotations

import re
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAYERS = ("Domain", "Platform", "BSP", "Service", "App")
ALLOWED = {
    "Domain": {"Domain"},
    "Platform": {"Platform"},
    "BSP": {"BSP", "Domain", "Platform"},
    "Service": {"Service", "Domain", "BSP", "Platform"},
    "App": set(LAYERS),
}
SOURCE_SUFFIXES = {".c", ".h"}

BLOCK_COMMENT = re.compile(r"/\*.*?\*/", re.DOTALL)
LINE_COMMENT = re.compile(r"//[^\n]*")
STRING_LITERAL = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'')
INCLUDE = re.compile(r"^\s*#\s*include\s*[<\"]([^>\"]+)[>\"]", re.MULTILINE)

FORBIDDEN_APIS = {
    "Domain": re.compile(
        r"\b(?:HAL_[A-Za-z0-9_]+|os(?:Kernel|Delay|Thread|Event|Mutex|Semaphore)[A-Za-z0-9_]*|"
        r"NVIC_[A-Za-z0-9_]+|SCB|SysTick|__(?:disable_irq|enable_irq|get_PRIMASK|set_PRIMASK|DSB|ISB))\b"
    ),
    "Service": re.compile(
        r"\b(?:HAL_[A-Za-z0-9_]+|os(?:Kernel|Delay|Thread|Event|Mutex|Semaphore)[A-Za-z0-9_]*|"
        r"NVIC_[A-Za-z0-9_]+|SCB|SysTick|__(?:disable_irq|enable_irq|get_PRIMASK|set_PRIMASK))\b"
    ),
}

FORBIDDEN_HEADER_MARKERS = {
    "Domain": ("stm32", "cmsis", "freertos", "main.h", "gpio.h", "tim.h", "usart.h", "adc.h", "spi.h"),
    "Service": ("stm32", "cmsis", "freertos", "main.h", "gpio.h", "tim.h", "usart.h", "adc.h", "spi.h"),
}

OLD_SYMBOLS = re.compile(
    r"\b(?:ChassisTasks_|ControlManager_|SystemMonitor_|ParamStore_)|"
    r"(?:chassis_tasks|control_manager|system_monitor|param_store)"
)
REMOVED_HEADERS = {"chassis_config.h"}


def strip_non_code(text: str) -> str:
    text = BLOCK_COMMENT.sub(" ", text)
    text = LINE_COMMENT.sub(" ", text)
    return STRING_LITERAL.sub('""', text)


def source_files(layer: str) -> list[Path]:
    base = ROOT / layer
    return sorted(path for path in base.rglob("*") if path.suffix.lower() in SOURCE_SUFFIXES)


def build_header_index() -> dict[str, set[str]]:
    index: dict[str, set[str]] = defaultdict(set)
    for layer in LAYERS:
        for header in (ROOT / layer).rglob("*.h"):
            index[header.name].add(layer)
            index[header.relative_to(ROOT).as_posix()].add(layer)
    return index


def include_layer(include: str, index: dict[str, set[str]]) -> str | None:
    normalized = include.replace("\\", "/")
    first = normalized.split("/", 1)[0]
    if first in LAYERS:
        return first
    matches = index.get(normalized) or index.get(Path(normalized).name)
    if matches and len(matches) == 1:
        return next(iter(matches))
    return None


def main() -> int:
    errors: list[str] = []
    header_index = build_header_index()

    for layer in LAYERS:
        for path in source_files(layer):
            relative = path.relative_to(ROOT).as_posix()
            raw = path.read_text(encoding="utf-8", errors="replace")
            without_comments = LINE_COMMENT.sub(" ", BLOCK_COMMENT.sub(" ", raw))
            for include in INCLUDE.findall(without_comments):
                normalized_include = include.replace("\\", "/")
                if Path(normalized_include).name in REMOVED_HEADERS:
                    errors.append(f"{relative}: removed legacy header {include}")
                target = include_layer(include, header_index)
                if target is not None and target not in ALLOWED[layer]:
                    errors.append(f"{relative}: {layer} must not include {target} header {include}")
                lowered = include.lower()
                if layer in FORBIDDEN_HEADER_MARKERS and any(
                    marker in lowered for marker in FORBIDDEN_HEADER_MARKERS[layer]
                ):
                    errors.append(f"{relative}: forbidden {layer} header {include}")

            code = strip_non_code(raw)
            pattern = FORBIDDEN_APIS.get(layer)
            if pattern is not None:
                match = pattern.search(code)
                if match:
                    line = code.count("\n", 0, match.start()) + 1
                    errors.append(f"{relative}:{line}: forbidden {layer} API {match.group(0)}")
            if OLD_SYMBOLS.search(code):
                match = OLD_SYMBOLS.search(code)
                assert match is not None
                line = code.count("\n", 0, match.start()) + 1
                errors.append(f"{relative}:{line}: removed legacy symbol/path {match.group(0)}")

    if errors:
        print("Architecture dependency check failed:")
        for error in errors:
            print(f"  - {error}")
        return 1

    print("Architecture dependency check passed (Domain/Platform/BSP/Service/App).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
