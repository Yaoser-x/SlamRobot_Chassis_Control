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
REQUIRED_ARTIFACTS = (
    "F407_V2.0.elf",
    "F407_V2.0.bin",
    "F407_V2.0.hex",
    "F407_V2.0.map",
)
ROS2_SLAM_BRANCH = "ARM"
ROS2_SLAM_RELEASE = "v0.6.0-rc2"
ROS2_SLAM_CANDIDATE_COMMIT = "c472bc63edc58f666444094cc8edfd8ef20675a2"
SEMVER_RE = re.compile(
    r"^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)(?:-([0-9A-Za-z.-]+))?$"
)


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def release_identity(version: str, requested_tag: str | None = None) -> tuple[str, str]:
    match = SEMVER_RE.fullmatch(version)
    if match is None:
        raise ValueError(f"VERSION is not supported SemVer: {version!r}")
    expected_tag = f"v{version}"
    if requested_tag is not None and requested_tag != expected_tag:
        raise ValueError(f"tag {requested_tag!r} does not match VERSION {version!r}")
    release_type = "prerelease" if match.group(4) is not None else "release"
    return expected_tag, release_type


def release_verification_state(
    release_type: str,
    missing_reports: list[str],
    raspberry_pi_uart_verified: bool,
) -> dict[str, object]:
    hil_complete = not missing_reports and raspberry_pi_uart_verified
    release_compatible = release_type == "release" and hil_complete
    software_package_complete = release_type == "prerelease" or release_compatible
    return {
        "software_status": "FEATURE_FROZEN"
        if release_type == "prerelease"
        else "RELEASED",
        "hardware_status": "HIL_VERIFIED" if hil_complete else "HIL_PENDING",
        "software_package_complete": software_package_complete,
        "hil_complete": hil_complete,
        "release_compatible": release_compatible,
        "complete": software_package_complete,
    }


def copy_with_hash(
    source: Path, destination: Path, bundle_root: Path, records: list[dict[str, object]]
) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    records.append(
        {
            "path": destination.relative_to(bundle_root).as_posix(),
            "bytes": destination.stat().st_size,
            "sha256": sha256(destination),
        }
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-dir", type=Path, default=ROOT / "build" / "Release")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--parameter-crc",
        required=True,
        help="runtime parameter identity CRC, e.g. 0x12345678",
    )
    parser.add_argument("--host-report", type=Path)
    parser.add_argument("--esp-report", type=Path)
    parser.add_argument("--dual-report", type=Path)
    parser.add_argument("--board-report", type=Path)
    parser.add_argument("--vehicle-report", type=Path)
    parser.add_argument("--evidence", type=Path, action="append", default=[])
    parser.add_argument(
        "--tag", help="annotated tag; defaults to v${VERSION} and must match VERSION"
    )
    parser.add_argument(
        "--allow-incomplete",
        action="store_true",
        help="allow a prerelease package to record missing HIL reports; never relaxes clean/tag/build identity",
    )
    args = parser.parse_args()

    if args.output_dir.exists():
        raise SystemExit("output directory already exists; choose a new path")
    status = git("status", "--porcelain")
    if status:
        raise SystemExit("release packaging requires a clean Git worktree")
    commit = git("rev-parse", "HEAD")
    version = (ROOT / "VERSION").read_text(encoding="utf-8").strip()
    try:
        expected_tag, release_type = release_identity(version, args.tag)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    tag = args.tag or expected_tag

    build_identity = args.build_dir / "generated" / "build_identity.h"
    identity_text = build_identity.read_text(encoding="utf-8")
    built_sha = re.search(r'#define F407_GIT_SHA "([0-9a-f]{40})"', identity_text)
    identity_valid = re.search(r"#define F407_BUILD_IDENTITY_VALID\s+1", identity_text)
    if not built_sha or built_sha.group(1) != commit or not identity_valid:
        raise SystemExit(
            "Release build identity is dirty, invalid, or does not match HEAD"
        )

    try:
        tag_target = git("rev-list", "-n", "1", tag)
    except subprocess.CalledProcessError:
        tag_target = ""
    if tag_target and tag_target != commit:
        raise SystemExit(f"tag {tag} does not point to HEAD")
    if not tag_target:
        raise SystemExit(f"annotated tag {tag} must exist before release packaging")

    report_args = {
        "host_uart_hil": args.host_report,
        "esp_hil": args.esp_report,
        "dual_link_hil": args.dual_report,
        "board_smoke": args.board_report,
        "vehicle_stop": args.vehicle_report,
    }
    missing_reports = [
        name for name, path in report_args.items() if path is None or not path.is_file()
    ]
    if missing_reports:
        if release_type == "release":
            raise SystemExit(
                "stable release is missing required reports: "
                + ", ".join(missing_reports)
            )
        if not args.allow_incomplete:
            raise SystemExit(
                "prerelease is missing HIL reports; pass --allow-incomplete to record HIL_PENDING: "
                + ", ".join(missing_reports)
            )

    for name in REQUIRED_ARTIFACTS:
        source = args.build_dir / name
        if not source.is_file():
            raise SystemExit(f"missing Release artifact: {source}")
    for source in args.evidence:
        if not source.is_file():
            raise SystemExit(f"missing evidence file: {source}")

    parameter_crc = int(args.parameter_crc, 0)
    if not 0 <= parameter_crc <= 0xFFFFFFFF:
        raise SystemExit("parameter CRC must fit uint32")
    raspberry_pi_uart_verified = False
    if args.host_report is not None and args.host_report.is_file():
        host_report = json.loads(args.host_report.read_text(encoding="utf-8"))
        assertions = host_report.get("assertions")
        raspberry_pi_uart_verified = (
            host_report.get("platform") == "raspberry-pi"
            and isinstance(assertions, dict)
            and bool(assertions)
            and all(value is True for value in assertions.values())
        )
    verification = release_verification_state(
        release_type, missing_reports, raspberry_pi_uart_verified
    )
    if release_type == "release" and not verification["release_compatible"]:
        raise SystemExit(
            "stable release requires a passing Raspberry Pi UART HIL report"
        )

    args.output_dir.mkdir(parents=True)
    records: list[dict[str, object]] = []
    for name in REQUIRED_ARTIFACTS:
        source = args.build_dir / name
        copy_with_hash(
            source, args.output_dir / "firmware" / name, args.output_dir, records
        )
    contract_files = [
        ROOT / "docs" / "protocol" / "upper-v3.schema.json",
        ROOT / "tests" / "fixtures" / "upper_v3_golden.json",
        ROOT / "App" / "config" / "robot_config.c",
        ROOT
        / "Service"
        / "parameter_management"
        / "internal"
        / "flash_parameter_image.h",
    ]
    for source in contract_files:
        copy_with_hash(
            source, args.output_dir / "contract" / source.name, args.output_dir, records
        )
    for source in sorted((ROOT / "docs" / "release").glob("*.md")):
        copy_with_hash(
            source,
            args.output_dir / "contract" / "release" / source.name,
            args.output_dir,
            records,
        )
    for name, source in report_args.items():
        if source is not None and source.is_file():
            copy_with_hash(
                source,
                args.output_dir / "reports" / f"{name}{source.suffix}",
                args.output_dir,
                records,
            )
    for source in args.evidence:
        copy_with_hash(
            source, args.output_dir / "evidence" / source.name, args.output_dir, records
        )
    manifest = {
        "firmware_version": version,
        "release_type": release_type,
        "software_status": verification["software_status"],
        "hardware_status": verification["hardware_status"],
        "git_commit": commit,
        "tag": tag,
        "hardware_revision": "0x00020000",
        "upper_protocol": 3,
        "parameter_schema": 4,
        "parameter_identity_crc32": f"0x{parameter_crc:08X}",
        "default_motor_enabled_mask": "0x06",
        "default_parameters_source": "App/config/robot_config.c",
        "golden_vector_sha256": sha256(
            ROOT / "tests" / "fixtures" / "upper_v3_golden.json"
        ),
        "compatibility": {
            "Ros2_Slam": {
                "branch": ROS2_SLAM_BRANCH,
                "release": ROS2_SLAM_RELEASE,
                "candidate_commit": ROS2_SLAM_CANDIDATE_COMMIT,
                "tested_commit": None,
                "compatible_commit": None,
                "status": "compatible"
                if verification["release_compatible"]
                else "candidate",
            }
        },
        "software_package_complete": verification["software_package_complete"],
        "hil_complete": verification["hil_complete"],
        "release_compatible": verification["release_compatible"],
        "complete": verification["complete"],
        "missing_reports": missing_reports,
        "created_unix_ms": int(time.time() * 1000),
        "files": records,
    }
    manifest_path = args.output_dir / "release-manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    records.append(
        {
            "path": manifest_path.name,
            "bytes": manifest_path.stat().st_size,
            "sha256": sha256(manifest_path),
        }
    )
    sums = "".join(f"{record['sha256']}  {record['path']}\n" for record in records)
    (args.output_dir / "SHA256SUMS").write_text(sums, encoding="utf-8")
    return 0 if manifest["complete"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
