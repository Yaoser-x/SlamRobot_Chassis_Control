#!/usr/bin/env python3
"""Validate Beta5 layer, contract, configuration, and Service DAG rules."""

from __future__ import annotations

import argparse
import re
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FINAL_LAYERS = ("Algorithm", "Platform", "BSP", "Service", "App")
MIGRATION_LAYERS = ("Domain",) + FINAL_LAYERS
SOURCE_SUFFIXES = {".c", ".h"}

ALLOWED = {
    "Algorithm": {"Algorithm"},
    "Domain": {"Domain"},
    "Platform": {"Platform"},
    "BSP": {"BSP", "Algorithm", "Domain", "Platform"},
    "Service": {"Service", "Algorithm", "Domain", "BSP", "Platform"},
    "App": set(MIGRATION_LAYERS),
}

FINAL_SERVICE_DEPENDENCIES = {
    "motion_control": {"command_management", "safety_management", "state_estimation", "power_management", "parameter_management"},
    "state_estimation": {"parameter_management"},
    "power_management": {"state_estimation", "parameter_management"},
    "safety_management": {"power_management", "state_estimation", "system_monitoring", "command_management"},
    "command_management": {"parameter_management"},
    "teleoperation": {"state_estimation", "line_following", "command_management", "parameter_management"},
    "line_following": {"safety_management", "command_management", "parameter_management"},
    "communication": {"command_management", "safety_management", "line_following", "parameter_management"},
    "parameter_management": set(),
    "system_monitoring": set(),
}

BLOCK_COMMENT = re.compile(r"/\*.*?\*/", re.DOTALL)
LINE_COMMENT = re.compile(r"//[^\n]*")
STRING_LITERAL = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'')
INCLUDE = re.compile(r"^\s*#\s*include\s*[<\"]([^>\"]+)[>\"]", re.MULTILINE)

FORBIDDEN_APIS = {
    "Algorithm": re.compile(
        r"\b(?:HAL_[A-Za-z0-9_]+|os(?:Kernel|Delay|Thread|Event|Mutex|Semaphore)[A-Za-z0-9_]*|"
        r"NVIC_[A-Za-z0-9_]+|SCB|SysTick|__(?:disable_irq|enable_irq|get_PRIMASK|set_PRIMASK|DSB|ISB))\b"
    ),
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
    "Algorithm": ("stm32", "cmsis", "freertos", "main.h", "gpio.h", "tim.h", "usart.h", "adc.h", "spi.h"),
    "Domain": ("stm32", "cmsis", "freertos", "main.h", "gpio.h", "tim.h", "usart.h", "adc.h", "spi.h"),
    "Service": ("stm32", "cmsis", "freertos", "main.h", "gpio.h", "tim.h", "usart.h", "adc.h", "spi.h"),
}

OLD_SYMBOLS = re.compile(
    r"\b(?:ChassisTasks_|ControlManager_|SystemMonitor_|ParamStore_)|"
    r"(?:chassis_tasks|control_manager|system_monitor(?!ing)|param_store)"
)
REMOVED_HEADERS = {"chassis_config.h"}

# Exact Beta4 debt. Entries may be removed as consumers migrate; new entries are
# forbidden. The final gate ignores this allowance and requires an empty debt.
LEGACY_PUBLIC_HEADER_LEAKS = {
    ("Service/chassis/chassis_feedback_guard.h", "encoder_driver.h"),
    ("Service/chassis/chassis_feedback_guard.h", "motor_driver.h"),
    ("Service/chassis/chassis_output_service.h", "adc_monitor.h"),
    ("Service/chassis/chassis_service.h", "motor_driver.h"),
    ("Service/chassis/chassis_speed_loop.h", "motor_driver.h"),
}

APP_BSP_INCLUDE_ALLOWLIST = {
    "App/app_init.c": {"led_status.h", "line_uart.h", "ssd1306.h"},
    "App/debug/commands/debug_cmd_control.c": {"motor_config.h"},
    "App/debug/commands/debug_cmd_current.c": {
        "adc_monitor.h", "adc_monitor_config.h", "chassis_layout.h", "motor_driver.h"
    },
    "App/debug/commands/debug_cmd_imu.c": {"imu_bmi270.h"},
    "App/debug/commands/debug_cmd_line.c": {"line_uart.h"},
    "App/debug/commands/debug_cmd_param.c": {"adc_monitor.h", "imu_bmi270.h"},
    "App/debug/commands/debug_cmd_system.c": {"i2c_bus_diagnostic.h"},
    "App/debug/commands/debug_rtos_report.c": {"debug_uart_transport.h"},
    "App/debug/commands/debug_system_status.c": {
        "adc_monitor.h", "adc_monitor_config.h", "chassis_layout.h", "encoder_driver.h", "imu_bmi270.h", "line_uart.h",
        "motor_driver.h"
    },
    "App/debug/debug_console_writer.c": {"debug_uart_transport.h"},
    "App/debug/debug_console_parser.h": {"motor_types.h"},
    "App/debug/debug_straight_telemetry.h": {"adc_monitor.h"},
    "App/debug/telemetry/debug_telemetry.c": {
        "adc_monitor.h", "encoder_driver.h", "imu_bmi270.h", "line_uart.h", "motor_driver.h"
    },
    "App/debug/telemetry/debug_telemetry_model.h": {
        "adc_monitor.h", "encoder_driver.h", "imu_bmi270.h", "line_uart.h", "motor_driver.h"
    },
    "App/debug/usart1_debug_console.c": {"debug_uart_transport.h"},
    "App/display/oled_text_renderer.c": {"ssd1306.h"},
    "App/display/oled_ui.c": {"bsp_config.h", "ssd1306.h"},
    "App/display/pages/oled_page_calibration.c": {"oled_font_8x16.h", "ssd1306.h"},
    "App/display/pages/oled_page_fault.c": {"oled_font_8x16.h", "ssd1306.h"},
    "App/display/pages/oled_page_runtime.c": {"oled_font_16x16.h", "oled_font_8x16.h", "ssd1306.h"},
    "App/display/pages/oled_page_selfcheck.c": {"oled_font_16x16.h", "oled_font_8x16.h", "ssd1306.h"},
    "App/display/pages/oled_page_welcome.c": {"oled_font_16x16.h", "oled_font_8x16.h", "ssd1306.h"},
    "App/adapters/esp12f_flash_bridge.c": {"esp12f_boot_control.h", "uart_bridge_transport.h"},
    "App/adapters/uart_callbacks.c": {"line_uart.h", "uart_bridge_transport.h"},
    "App/tasks/task_esp12f.c": {"bsp_config.h"},
    "App/tasks/task_imu.c": {"bsp_config.h"},
    "App/tasks/task_led.c": {"bsp_config.h", "led_status.h"},
    "App/tasks/task_line.c": {"bsp_config.h", "line_uart.h"},
    "App/tasks/task_motor.c": {"bsp_config.h"},
    "App/tasks/task_oled.c": {"bsp_config.h"},
    "App/tasks/task_ps2.c": {"bsp_config.h"},
    "App/tasks/task_safety.c": {"bsp_config.h"},
    "App/tasks/task_upper_comm.c": {"bsp_config.h"},
}

LEGACY_CONFIG_CONSUMERS = {
    "App/debug/commands/debug_cmd_control.c", "App/debug/commands/debug_cmd_current.c",
    "App/debug/commands/debug_system_status.c", "App/display/oled_ui.c", "App/tasks/task_esp12f.c",
    "App/tasks/task_imu.c", "App/tasks/task_led.c", "App/tasks/task_line.c", "App/tasks/task_motor.c",
    "App/tasks/task_oled.c", "App/tasks/task_ps2.c", "App/tasks/task_safety.c", "App/tasks/task_upper_comm.c",
    "BSP/adc/adc_monitor.c", "BSP/encoder/encoder_driver.c", "BSP/motor/motor_driver.c", "BSP/oled/ssd1306.c",
    "Service/chassis/chassis_feedback_guard.c", "Service/chassis/chassis_maintenance_service.c",
    "Service/chassis/chassis_output_service.c", "Service/chassis/chassis_param_sync.c",
    "Service/chassis/chassis_service.c", "Service/chassis/chassis_speed_loop.c",
    "Service/chassis/chassis_target_planner.c", "Service/chassis/chassis_test_mode.c",
    "Service/communication/communication_command_router.c", "Service/communication/esp12f_service.c",
    "Service/communication/upper_uart_service.c", "Service/control/control_service.c",
    "Service/control/line_control_service.c", "Service/control/ps2_control_service.c",
    "Service/parameter_management/param_service.c",
    "Service/safety_management/battery_guard.c", "Service/motion_control/internal/current_guard.c",
    "Service/telemetry/system_snapshot_service.c",
}


def strip_non_code(text: str) -> str:
    text = BLOCK_COMMENT.sub(" ", text)
    text = LINE_COMMENT.sub(" ", text)
    return STRING_LITERAL.sub('""', text)


def source_files(root: Path, layer: str) -> list[Path]:
    base = root / layer
    if not base.exists():
        return []
    return sorted(path for path in base.rglob("*") if path.suffix.lower() in SOURCE_SUFFIXES)


def build_header_index(root: Path) -> dict[str, set[str]]:
    index: dict[str, set[str]] = defaultdict(set)
    for layer in MIGRATION_LAYERS:
        base = root / layer
        if not base.exists():
            continue
        for header in base.rglob("*.h"):
            index[header.name].add(layer)
            index[header.relative_to(root).as_posix()].add(layer)
    return index


def build_internal_header_index(root: Path) -> dict[str, set[str]]:
    index: dict[str, set[str]] = defaultdict(set)
    service_root = root / "Service"
    if not service_root.exists():
        return index
    for header in service_root.glob("*/internal/**/*.h"):
        relative = header.relative_to(root).as_posix()
        owner = header.relative_to(service_root).parts[0]
        index[header.name].add(owner)
        index[relative].add(owner)
    return index


def include_layer(include: str, index: dict[str, set[str]]) -> str | None:
    normalized = include.replace("\\", "/")
    first = normalized.split("/", 1)[0]
    if first in MIGRATION_LAYERS:
        return first
    matches = index.get(normalized) or index.get(Path(normalized).name)
    if matches and len(matches) == 1:
        return next(iter(matches))
    return None


def _canonical_cycle(cycle: list[str]) -> tuple[str, ...]:
    body = cycle[:-1]
    rotations = [tuple(body[index:] + body[:index]) for index in range(len(body))]
    canonical = min(rotations)
    return canonical + (canonical[0],)


def _find_cycles(graph: dict[str, set[str]]) -> list[tuple[str, ...]]:
    state = {node: 0 for node in graph}
    stack: list[str] = []
    cycles: set[tuple[str, ...]] = set()

    def visit(node: str) -> None:
        state[node] = 1
        stack.append(node)
        for target in sorted(graph[node]):
            if state[target] == 0:
                visit(target)
            elif state[target] == 1:
                start = stack.index(target)
                cycles.add(_canonical_cycle(stack[start:] + [target]))
        stack.pop()
        state[node] = 2

    for node in sorted(graph):
        if state[node] == 0:
            visit(node)
    return sorted(cycles)


def final_service_graph_errors(root: Path) -> list[str]:
    service_root = root / "Service"
    errors: list[str] = []
    header_owners: dict[str, set[str]] = defaultdict(set)
    graph = {node: set() for node in FINAL_SERVICE_DEPENDENCIES}

    for node in graph:
        base = service_root / node
        if not base.is_dir():
            errors.append(f"Service/{node}: required final capability directory is missing")
            continue
        for header in base.rglob("*.h"):
            relative = header.relative_to(root).as_posix()
            header_owners[header.name].add(node)
            header_owners[relative].add(node)

    for node in graph:
        base = service_root / node
        if not base.is_dir():
            continue
        for path in source_files(root, "Service"):
            relative_path = path.relative_to(service_root)
            if not relative_path.parts or relative_path.parts[0] != node:
                continue
            raw = path.read_text(encoding="utf-8", errors="replace")
            for include in INCLUDE.findall(LINE_COMMENT.sub(" ", BLOCK_COMMENT.sub(" ", raw))):
                normalized = include.replace("\\", "/")
                owners = header_owners.get(normalized) or header_owners.get(Path(normalized).name)
                if owners and len(owners) == 1:
                    target = next(iter(owners))
                    if target != node:
                        graph[node].add(target)

    for node, targets in graph.items():
        for target in sorted(targets - FINAL_SERVICE_DEPENDENCIES[node]):
            errors.append(f"Service dependency not allowed: {node} -> {target}")
    for cycle in _find_cycles(graph):
        errors.append(f"Service dependency cycle: {' -> '.join(cycle)}")
    return errors


def analyze(root: Path, final: bool = False) -> list[str]:
    root = Path(root).resolve()
    errors: list[str] = []
    header_index = build_header_index(root)
    internal_header_index = build_internal_header_index(root)

    if final:
        if (root / "Domain").exists():
            errors.append("Domain/: legacy layer must be removed before Beta5 final")
        if not (root / "Algorithm").is_dir():
            errors.append("Algorithm/: required final layer is missing")

    for layer in MIGRATION_LAYERS:
        for path in source_files(root, layer):
            relative_path = path.relative_to(root)
            relative = relative_path.as_posix()
            raw = path.read_text(encoding="utf-8", errors="replace")
            without_comments = LINE_COMMENT.sub(" ", BLOCK_COMMENT.sub(" ", raw))
            for include in INCLUDE.findall(without_comments):
                normalized = include.replace("\\", "/")
                include_name = Path(normalized).name
                if include_name in REMOVED_HEADERS:
                    errors.append(f"{relative}: removed legacy header {include}")

                target = include_layer(include, header_index)
                if final and target == "Domain":
                    errors.append(f"{relative}: final code must not include Domain header {include}")
                elif target is not None and target not in ALLOWED[layer]:
                    errors.append(f"{relative}: {layer} must not include {target} header {include}")

                internal_owners = internal_header_index.get(normalized) or internal_header_index.get(include_name)
                if internal_owners:
                    service_owner = relative_path.parts[1] if layer == "Service" and len(relative_path.parts) > 1 else None
                    if service_owner not in internal_owners:
                        owners = ", ".join(sorted(internal_owners))
                        errors.append(f"{relative}: private Service internal header {include} belongs to {owners}")

                if layer == "App" and target == "BSP":
                    allowed = APP_BSP_INCLUDE_ALLOWLIST.get(relative, set())
                    if include_name not in allowed:
                        errors.append(f"{relative}: App BSP include {include} is not in the exact allowlist")

                public_service_header = layer == "Service" and path.suffix.lower() == ".h" and "internal" not in relative_path.parts
                if public_service_header and target in {"BSP", "Platform"}:
                    debt = (relative, include_name) in LEGACY_PUBLIC_HEADER_LEAKS
                    if final or not debt:
                        errors.append(f"{relative}: public Service header leaks {target} header {include}")

                if include_name in {"control_config.h", "bsp_config.h"}:
                    if final or relative not in LEGACY_CONFIG_CONSUMERS:
                        errors.append(f"{relative}: legacy mixed configuration include {include}")

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
            match = OLD_SYMBOLS.search(code)
            if match:
                line = code.count("\n", 0, match.start()) + 1
                errors.append(f"{relative}:{line}: removed legacy symbol/path {match.group(0)}")

    if final:
        errors.extend(final_service_graph_errors(root))
    return sorted(set(errors))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--final", action="store_true", help="require the completed Beta5 layout with no migration debt")
    return parser.parse_args()


def main(final: bool | None = None) -> int:
    if final is None:
        final = parse_args().final
    errors = analyze(ROOT, final=final)
    if errors:
        print("Architecture dependency check failed:")
        for error in errors:
            print(f"  - {error}")
        return 1
    mode = "final" if final else "migration"
    print(f"Architecture dependency check passed ({mode}: layers, contracts, config, Service DAG).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
