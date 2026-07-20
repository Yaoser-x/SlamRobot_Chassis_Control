import json
import subprocess
import sys
from pathlib import Path


def main() -> int:
    generator = Path(sys.argv[1])
    fixture = Path(sys.argv[2])
    generated = json.loads(subprocess.check_output([generator], text=True))
    expected = json.loads(fixture.read_text(encoding="utf-8"))
    if generated != expected:
        raise AssertionError("upper_v3 golden frames are stale; regenerate from firmware encoder")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
