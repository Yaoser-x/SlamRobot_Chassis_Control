#!/usr/bin/env python3
"""Validate final runtime ownership and Upper Protocol v3 dependency boundaries."""

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
IMU_LIFECYCLE_API = re.compile(
    r"\bBmi270Driver_(?:SetEnabled|SetProfile|ProbeNow|ConfigNow|Diagnose)\s*\("
)
SERVICE_GETTER = re.compile(r"\b[A-Z][A-Za-z0-9]*(?:Management|Control|Estimation|Monitoring)_Get[A-Za-z0-9_]*\s*\(")
APP_DECISION_API = re.compile(r"\b[A-Z][A-Za-z0-9]*(?:_Set|_Clear|_Enable|_Disable)[A-Za-z0-9_]*\s*\(")
SAFETY_EXTERNAL_SERVICE_CALL = re.compile(
    r"\b(?:CommandManagement|PowerManagement|StateEstimation|SystemMonitoring|MotorDriver)_[A-Za-z0-9_]+\s*\("
)
MOTION_EXTERNAL_SERVICE_CALL = re.compile(
    r"\b(?:SafetyManagement|CommandManagement|PowerManagement|StateEstimation|ParameterManagement)_[A-Za-z0-9_]+\s*\("
)
SAFETY_MUTATOR = re.compile(
    r"\bSafetyManagement_(?:SetEmergencyStop|SetFaultStop|ClearLatchedFaults|"
    r"LatchEncoderFeedbackFault|BeginMaintenance|EndMaintenance)\s*\("
)
REMOVED_RUNTIME_ENTRY = re.compile(
    r"\b(?:SafetyManagement_Update|MotionControl_Step|MotionControl_StepWithPeriod|"
    r"MotionControl_BeginMaintenance|MotionControl_EndMaintenance)\s*\("
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
            for match in IMU_LIFECYCLE_API.finditer(code):
                final_owner = relative == "Service/state_estimation/state_estimation_service.c"
                if not final_owner:
                    line = code.count("\n", 0, match.start()) + 1
                    errors.append(
                        f"{relative}:{line}: BMI270 lifecycle API is owned only by State Estimation"
                    )

    communication = root / "Service" / "communication"
    for path in communication.rglob("*"):
        if path.suffix.lower() not in SOURCE_SUFFIXES:
            continue
        relative = path.relative_to(root).as_posix()
        code = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
        if "parameter_management" in code.lower():
            errors.append(f"{relative}: Communication must receive parameter identity through the publish DTO")
        if re.search(r"\b(?:SafetyManagement|LineFollowing)_", code):
            errors.append(f"{relative}: Communication operations must be dispatched by App, not call business Services")
        if relative.endswith("communication_operation_mailbox.c") and re.search(r"\b(?:malloc|calloc|realloc|free)\s*\(", code):
            errors.append(f"{relative}: operation mailbox must remain fixed-capacity and allocation-free")

    dispatcher_path = communication / "internal" / "remote_command_dispatcher.c"
    if dispatcher_path.is_file():
        dispatcher_code = strip_comments(dispatcher_path.read_text(encoding="utf-8", errors="replace"))
        if "COMMAND_INTENT_REMOTE_DISABLE" not in dispatcher_code or "CommandManagement_ApplyIntent" not in dispatcher_code:
            errors.append("Service/communication/internal/remote_command_dispatcher.c: remote disable must pass rearm owner")

    command_root = root / "Service" / "command_management"
    for path in command_root.rglob("*"):
        if path.suffix.lower() in SOURCE_SUFFIXES:
            code = strip_comments(path.read_text(encoding="utf-8", errors="replace")).lower()
            if "robot_link_protocol" in code or "communication_session" in code:
                errors.append(f"{path.relative_to(root).as_posix()}: Command Management must not understand wire sessions")

    safety_root = root / "Service" / "safety_management"
    for path in safety_root.rglob("*"):
        if path.suffix.lower() in SOURCE_SUFFIXES:
            code = strip_comments(path.read_text(encoding="utf-8", errors="replace")).lower()
            if "robot_link_protocol" in code or "_transport.h" in code:
                errors.append(f"{path.relative_to(root).as_posix()}: Safety Management must not depend on protocol transport")

    safety_service_path = safety_root / "safety_management_service.c"
    if safety_service_path.is_file():
        safety_service_code = strip_comments(safety_service_path.read_text(encoding="utf-8", errors="replace"))
        for match in SAFETY_EXTERNAL_SERVICE_CALL.finditer(safety_service_code):
            line = safety_service_code.count("\n", 0, match.start()) + 1
            errors.append(
                f"Service/safety_management/safety_management_service.c:{line}: "
                "Safety must consume App-composed facts and return decisions"
            )

    motion_root = root / "Service" / "motion_control"
    for path in motion_root.rglob("*.c"):
        relative = path.relative_to(root).as_posix()
        code = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
        for match in MOTION_EXTERNAL_SERVICE_CALL.finditer(code):
            line = code.count("\n", 0, match.start()) + 1
            errors.append(f"{relative}:{line}: Motion must consume DTO facts and return events")

    for scan_root in (root / "App", root / "Service"):
        if not scan_root.exists():
            continue
        for path in scan_root.rglob("*.c"):
            relative = path.relative_to(root).as_posix()
            code = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
            for match in REMOVED_RUNTIME_ENTRY.finditer(code):
                line = code.count("\n", 0, match.start()) + 1
                errors.append(f"{relative}:{line}: removed pull-based runtime entrypoint is forbidden")
            for match in SAFETY_MUTATOR.finditer(code):
                if relative.startswith("Service/safety_management/") or relative == (
                    "App/composition/safety_workflow_coordinator.c"
                ):
                    continue
                line = code.count("\n", 0, match.start()) + 1
                errors.append(f"{relative}:{line}: Safety mutations must be routed by AppSafetyWorkflow")

    gate_writers: list[str] = []
    for scan_root in (root / "App", root / "Service"):
        if not scan_root.exists():
            continue
        for path in scan_root.rglob("*.c"):
            relative = path.relative_to(root).as_posix()
            code = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
            if re.search(r"\bCommandManagement_SetMotionGate\s*\(", code):
                gate_writers.append(relative)
    allowed_gate_writers = {
        "App/composition/safety_workflow_coordinator.c",
        "Service/command_management/command_management_service.c",
    }
    for relative in sorted(set(gate_writers) - allowed_gate_writers):
        errors.append(f"{relative}: Command motion gate may only be synchronized by AppSafetyWorkflow")

    for relative in (
        "Service/communication/internal/robot_link_protocol.c",
        "Service/communication/internal/telemetry_encoder.c",
    ):
        path = root / relative
        if not path.is_file():
            continue
        code = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
        if SERVICE_GETTER.search(code):
            errors.append(f"{relative}: codec and telemetry encoder must consume only passed value objects")

    host_path = communication / "host_communication_service.c"
    wireless_path = communication / "wireless_communication_service.c"
    if host_path.is_file():
        host_code = strip_comments(host_path.read_text(encoding="utf-8"))
        if re.search(r"\bWirelessCommunication_", host_code):
            errors.append("Service/communication/host_communication_service.c: Host must not call Wireless Communication")
    if wireless_path.is_file():
        wireless_code = strip_comments(wireless_path.read_text(encoding="utf-8"))
        if re.search(r"\bHostCommunication_", wireless_code):
            errors.append("Service/communication/wireless_communication_service.c: Wireless must not call Host Communication")

    collector_path = root / "App" / "composition" / "system_publish_snapshot_collector.c"
    if collector_path.is_file():
        collector = strip_comments(collector_path.read_text(encoding="utf-8", errors="replace"))
        if APP_DECISION_API.search(collector):
            errors.append("App/composition/system_publish_snapshot_collector.c: collector must be read-only")

    runtime_path = root / "App" / "composition" / "chassis_runtime_coordinator.c"
    if runtime_path.is_file():
        runtime = strip_comments(runtime_path.read_text(encoding="utf-8", errors="replace"))
        capability_apply = runtime.find("ControlModeCoordinator_ApplyCapabilityMask(")
        source_select = runtime.find("CommandManagement_GetActiveSource(", capability_apply + 1)
        if capability_apply < 0 or source_select < 0 or capability_apply > source_select:
            errors.append(
                "App/composition/chassis_runtime_coordinator.c: Safety capability mask must be applied before source selection"
            )

    ps2_task_path = root / "App" / "tasks" / "task_ps2.c"
    if ps2_task_path.is_file():
        ps2_task = strip_comments(ps2_task_path.read_text(encoding="utf-8", errors="replace"))
        restored_line = re.search(
            r"CONTROL_MODE_EVENT_RESTORED_LINE\)(.*?)(?:else\s+if|OperatorActionRouter_Handle)",
            ps2_task,
            flags=re.DOTALL,
        )
        if restored_line is None or "LineFollowing_Enable(0U)" not in restored_line.group(1):
            errors.append("App/tasks/task_ps2.c: LINE mode restoration must submit a neutral intent")
        elif "LineFollowing_Enable(1U)" in restored_line.group(1):
            errors.append("App/tasks/task_ps2.c: LINE mode restoration must not automatically resume motion")
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
    print("Service ownership check passed (runtime owners and Upper Protocol v3 boundaries).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
