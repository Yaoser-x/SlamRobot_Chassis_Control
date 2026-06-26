#!/usr/bin/env python3
"""Validate the generated ESP12F gzip homepage asset."""

from __future__ import annotations

import gzip
import re
import sys
from pathlib import Path


SKETCH_DIR = Path(__file__).resolve().parent.parent
SKETCH_PATH = SKETCH_DIR / "F407_ESP12F.ino"
HEADER_PATH = SKETCH_DIR / "html_page_gz.h"


def extract_html(sketch_text: str) -> bytes:
    match = re.search(
        r'static const char HTML_PAGE\[\] PROGMEM = R"raw\(\r?\n'
        r"(.*?)"
        r'\r?\n\)raw";',
        sketch_text,
        flags=re.DOTALL,
    )
    if match is None:
        raise ValueError("HTML_PAGE source literal not found")
    return (match.group(1) + "\n").encode("utf-8")


def extract_gzip(header_text: str) -> bytes:
    match = re.search(
        r"HTML_PAGE_GZ\[\]\s+PROGMEM\s*=\s*\{(.*?)\};",
        header_text,
        flags=re.DOTALL,
    )
    if match is None:
        raise ValueError("HTML_PAGE_GZ array not found")
    values = re.findall(r"0x([0-9A-Fa-f]{2})", match.group(1))
    if not values:
        raise ValueError("HTML_PAGE_GZ array is empty")
    return bytes(int(value, 16) for value in values)


def main() -> int:
    if not HEADER_PATH.exists():
        print(f"missing generated asset: {HEADER_PATH}", file=sys.stderr)
        return 1

    sketch_text = SKETCH_PATH.read_text(encoding="utf-8")
    source = extract_html(sketch_text)
    compressed = extract_gzip(HEADER_PATH.read_text(encoding="utf-8"))
    decoded = gzip.decompress(compressed)

    if decoded != source:
        print("generated gzip asset does not match HTML_PAGE", file=sys.stderr)
        return 1
    if not decoded.rstrip().endswith(b"</html>"):
        print("decoded homepage is incomplete", file=sys.stderr)
        return 1
    if len(compressed) >= len(source) // 2:
        print("gzip asset did not reduce the homepage below 50%", file=sys.stderr)
        return 1
    if 'http.sendHeader("Content-Encoding", "gzip");' not in sketch_text:
        print("homepage response is missing Content-Encoding: gzip", file=sys.stderr)
        return 1
    if "HTML_PAGE_GZ_LEN" not in sketch_text:
        print("homepage response is missing the explicit gzip length", file=sys.stderr)
        return 1

    print(f"html={len(source)} bytes gzip={len(compressed)} bytes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
