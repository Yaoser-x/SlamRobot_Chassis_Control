#!/usr/bin/env python3
"""Build the ESP image with a checked source identity; never upload or write settings."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

SKETCH = Path(__file__).resolve().parents[1]
ROOT = SKETCH.parents[2]
DEFAULT_FQBN = "esp8266:esp8266:generic:eesz=4M,FlashMode=dout,CrystalFreq=26,baud=115200,ResetMethod=ck"


def verify_embedded_identity(images, commit, web_sha256):
    if (
        not images
        or not re.fullmatch(r"[0-9a-f]{40}", commit)
        or not re.fullmatch(r"[0-9a-f]{64}", web_sha256)
    ):
        return False
    for path in images:
        binary = path.read_bytes()
        if (
            commit.encode("ascii") not in binary
            or web_sha256.encode("ascii") not in binary
        ):
            return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--arduino-cli", default="arduino-cli")
    parser.add_argument("--config-file", type=Path)
    parser.add_argument("--fqbn", default=DEFAULT_FQBN)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise SystemExit(
            "output must be a new directory to preserve earlier build evidence"
        )
    commit = subprocess.check_output(
        [
            "git",
            "-c",
            f"safe.directory={ROOT.as_posix()}",
            "-C",
            str(ROOT),
            "rev-parse",
            "HEAD",
        ],
        text=True,
    ).strip()
    dirty = bool(
        subprocess.check_output(
            [
                "git",
                "-c",
                f"safe.directory={ROOT.as_posix()}",
                "-C",
                str(ROOT),
                "status",
                "--porcelain",
                "--untracked-files=all",
            ],
            text=True,
        ).strip()
    )
    if not re.fullmatch(r"[0-9a-f]{40}", commit):
        raise SystemExit("invalid git identity")
    if output.is_relative_to(ROOT):
        ignored = (
            subprocess.run(
                [
                    "git",
                    "-c",
                    f"safe.directory={ROOT.as_posix()}",
                    "-C",
                    str(ROOT),
                    "check-ignore",
                    "--quiet",
                    str(output),
                ]
            ).returncode
            == 0
        )
        if not ignored:
            raise SystemExit(
                "build output must be outside the source repository or git-ignored"
            )
    output.mkdir(parents=True)
    include = output / "include"
    include.mkdir()
    (include / "esp_build_identity_generated.h").write_text(
        f'#define ESP_BUILD_GIT_SHA "{commit}"\n#define ESP_BUILD_DIRTY {int(dirty)}\n#define ESP_BUILD_IDENTITY_VALID 1\n',
        encoding="utf-8",
    )
    command = [args.arduino_cli]
    if args.config_file:
        command += ["--config-file", str(args.config_file.resolve())]
    command += [
        "compile",
        "--fqbn",
        args.fqbn,
        "--build-path",
        str(output / "build"),
        "--output-dir",
        str(output / "image"),
        "--build-property",
        f'compiler.cpp.extra_flags="-I{include.as_posix()}"',
        str(SKETCH),
    ]
    with (output / "build.log").open("w", encoding="utf-8") as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    after_commit = subprocess.check_output(
        [
            "git",
            "-c",
            f"safe.directory={ROOT.as_posix()}",
            "-C",
            str(ROOT),
            "rev-parse",
            "HEAD",
        ],
        text=True,
    ).strip()
    after_dirty = bool(
        subprocess.check_output(
            [
                "git",
                "-c",
                f"safe.directory={ROOT.as_posix()}",
                "-C",
                str(ROOT),
                "status",
                "--porcelain",
                "--untracked-files=all",
            ],
            text=True,
        ).strip()
    )
    identity_consistent = after_commit == commit and after_dirty == dirty
    image_paths = sorted((output / "image").glob("*.bin"))
    web_hash = re.search(
        r'HTML_PAGE_SHA256 "([0-9a-f]{64})"',
        (SKETCH / "html_page_gz.h").read_text(encoding="utf-8"),
    )
    identity_embedded = verify_embedded_identity(
        image_paths, commit, web_hash.group(1) if web_hash else ""
    )
    exit_code = result.returncode or (
        0 if identity_consistent and identity_embedded else 2
    )
    images = [
        {
            "file": str(path.relative_to(output)),
            "bytes": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
        for path in image_paths
    ]
    (output / "manifest.json").write_text(
        json.dumps(
            {
                "commit": commit,
                "dirty": dirty,
                "fqbn": args.fqbn,
                "exit_code": exit_code,
                "identity_consistent": identity_consistent,
                "identity_embedded": identity_embedded,
                "images": images,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
