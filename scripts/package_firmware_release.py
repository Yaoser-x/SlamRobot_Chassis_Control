"""Create a traceable firmware release bundle from a clean Release build."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REQUIRED_ARTIFACTS = ("F407_V2.0.elf", "F407_V2.0.bin", "F407_V2.0.hex", "F407_V2.0.map")


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def copy_with_hash(source: Path, destination: Path, bundle_root: Path, records: list[dict[str, object]]) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    records.append({"path": destination.relative_to(bundle_root).as_posix(), "bytes": destination.stat().st_size, "sha256": sha256(destination)})


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-dir", type=Path, default=ROOT / "build" / "Release")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--parameter-crc", required=True, help="runtime parameter identity CRC, e.g. 0x12345678")
    parser.add_argument("--host-report", type=Path)
    parser.add_argument("--esp-report", type=Path)
    parser.add_argument("--dual-report", type=Path)
    parser.add_argument("--board-report", type=Path)
    parser.add_argument("--vehicle-report", type=Path)
    parser.add_argument("--evidence", type=Path, action="append", default=[])
    parser.add_argument("--tag", default="v1.0.0-rc1")
    parser.add_argument("--allow-incomplete", action="store_true")
    args = parser.parse_args()

    if args.output_dir.exists():
        raise SystemExit("output directory already exists; choose a new path")
    status = git("status", "--porcelain")
    if status and not args.allow_incomplete:
        raise SystemExit("release packaging requires a clean Git worktree")
    commit = git("rev-parse", "HEAD")
    version = (ROOT / "VERSION").read_text(encoding="utf-8").strip()
    if version != "1.0.0-rc1":
        raise SystemExit(f"unexpected VERSION {version!r}")

    build_identity = args.build_dir / "generated" / "build_identity.h"
    identity_text = build_identity.read_text(encoding="utf-8")
    built_sha = re.search(r'#define F407_GIT_SHA "([0-9a-f]{40})"', identity_text)
    identity_valid = re.search(r"#define F407_BUILD_IDENTITY_VALID\s+1", identity_text)
    if not built_sha or built_sha.group(1) != commit or not identity_valid:
        raise SystemExit("Release build identity is dirty, invalid, or does not match HEAD")

    try:
        tag_target = git("rev-list", "-n", "1", args.tag)
    except subprocess.CalledProcessError:
        tag_target = ""
    if tag_target and tag_target != commit:
        raise SystemExit(f"tag {args.tag} does not point to HEAD")
    if not tag_target and not args.allow_incomplete:
        raise SystemExit(f"annotated tag {args.tag} must exist before final packaging")

    report_args = {
        "host_uart_hil": args.host_report,
        "esp_hil": args.esp_report,
        "dual_link_hil": args.dual_report,
        "board_smoke": args.board_report,
        "vehicle_stop": args.vehicle_report,
    }
    missing_reports = [name for name, path in report_args.items() if path is None or not path.is_file()]
    if missing_reports and not args.allow_incomplete:
        raise SystemExit("missing required reports: " + ", ".join(missing_reports))

    args.output_dir.mkdir(parents=True)
    records: list[dict[str, object]] = []
    for name in REQUIRED_ARTIFACTS:
        source = args.build_dir / name
        if not source.is_file():
            raise SystemExit(f"missing Release artifact: {source}")
        copy_with_hash(source, args.output_dir / "firmware" / name, args.output_dir, records)
    contract_files = [
        ROOT / "docs" / "protocol" / "upper-v3.schema.json",
        ROOT / "tests" / "fixtures" / "upper_v3_golden.json",
        ROOT / "App" / "config" / "robot_config.c",
        ROOT / "Service" / "parameter_management" / "internal" / "flash_parameter_image.h",
    ]
    for source in contract_files:
        copy_with_hash(source, args.output_dir / "contract" / source.name, args.output_dir, records)
    for source in sorted((ROOT / "docs" / "release").glob("*.md")):
        copy_with_hash(source, args.output_dir / "contract" / "release" / source.name, args.output_dir, records)
    for name, source in report_args.items():
        if source is not None and source.is_file():
            copy_with_hash(source, args.output_dir / "reports" / f"{name}{source.suffix}", args.output_dir, records)
    for source in args.evidence:
        if not source.is_file():
            raise SystemExit(f"missing evidence file: {source}")
        copy_with_hash(source, args.output_dir / "evidence" / source.name, args.output_dir, records)

    parameter_crc = int(args.parameter_crc, 0)
    if not 0 <= parameter_crc <= 0xFFFFFFFF:
        raise SystemExit("parameter CRC must fit uint32")
    raspberry_pi_uart_verified = False
    if args.host_report is not None and args.host_report.is_file():
        host_report = json.loads(args.host_report.read_text(encoding="utf-8"))
        raspberry_pi_uart_verified = host_report.get("platform") == "raspberry-pi" and all(
            host_report.get("assertions", {}).values()
        )
    manifest = {
        "firmware_version": version,
        "git_commit": commit,
        "tag": args.tag if tag_target else None,
        "hardware_revision": "0x00020000",
        "upper_protocol": 3,
        "parameter_schema": 4,
        "parameter_identity_crc32": f"0x{parameter_crc:08X}",
        "default_motor_enabled_mask": "0x06",
        "default_parameters_source": "contract/robot_config.c",
        "golden_vector_sha256": sha256(ROOT / "tests" / "fixtures" / "upper_v3_golden.json"),
        "compatibility": {"Ros2_Slam": {"version": "v0.4.0", "status": "compatible" if raspberry_pi_uart_verified else "candidate"}},
        "complete": not missing_reports and bool(tag_target) and not status,
        "missing_reports": missing_reports,
        "created_unix_ms": int(time.time() * 1000),
        "files": records,
    }
    manifest_path = args.output_dir / "release-manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    records.append({"path": manifest_path.name, "bytes": manifest_path.stat().st_size, "sha256": sha256(manifest_path)})
    sums = "".join(f"{record['sha256']}  {record['path']}\n" for record in records)
    (args.output_dir / "SHA256SUMS").write_text(sums, encoding="utf-8")
    return 0 if manifest["complete"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
