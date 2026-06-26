#!/usr/bin/env python3
"""Generate a deterministic PROGMEM gzip asset from HTML_PAGE."""

from __future__ import annotations

import gzip
from pathlib import Path

from check_html_gz import HEADER_PATH, SKETCH_PATH, extract_html


def format_header(compressed: bytes) -> str:
    rows = []
    for offset in range(0, len(compressed), 16):
        chunk = compressed[offset : offset + 16]
        rows.append("  " + ", ".join(f"0x{value:02X}" for value in chunk) + ",")

    return "\n".join(
        [
            "#ifndef HTML_PAGE_GZ_H",
            "#define HTML_PAGE_GZ_H",
            "",
            "#include <stddef.h>",
            "#include <stdint.h>",
            "",
            "static const uint8_t HTML_PAGE_GZ[] PROGMEM = {",
            *rows,
            "};",
            "",
            "static const size_t HTML_PAGE_GZ_LEN = sizeof(HTML_PAGE_GZ);",
            "",
            "#endif /* HTML_PAGE_GZ_H */",
            "",
        ]
    )


def main() -> None:
    source = extract_html(SKETCH_PATH.read_text(encoding="utf-8"))
    compressed = gzip.compress(source, compresslevel=9, mtime=0)
    HEADER_PATH.write_text(format_header(compressed), encoding="utf-8", newline="\n")
    print(f"generated {HEADER_PATH.name}: html={len(source)} gzip={len(compressed)}")


if __name__ == "__main__":
    main()
