#!/usr/bin/env python3
"""Rebuild camera_index.h from web/index_ov2640.html.

The ESP32 serves the web page from a gzipped byte array in camera_index.h,
which can't be edited by hand. Edit web/index_ov2640.html instead, then run:

    python3 web/build_camera_index.py

from the ESP32_CameraServer_AP_20220120 folder (or anywhere: paths are
relative to this script) and re-upload the ESP32 code.
Only the OV2640 page is rebuilt (the Elegoo car's camera); the OV3660 page is left as is.
"""
import gzip
import pathlib
import re

HERE = pathlib.Path(__file__).resolve().parent
HTML = HERE / "index_ov2640.html"
HEADER = HERE.parent / "camera_index.h"
NAME = "index_ov2640_html_gz"


def main():
    data = gzip.compress(HTML.read_bytes(), compresslevel=9, mtime=0)

    rows = []
    for i in range(0, len(data), 16):
        rows.append("    " + ", ".join(f"0x{b:02X}" for b in data[i:i + 16]))
    block = (
        f"//File: index_ov2640.html.gz, Size: {len(data)}\n"
        f"#define {NAME}_len ({len(data)})\n"
        f"const uint8_t {NAME}[] = {{\n" + ",\n".join(rows) + "\n};\n"
    )

    src = HEADER.read_text(encoding="utf-8")
    pattern = re.compile(r"//File: index_ov2640\.html\.gz.*?\n#define " + NAME + r"_len.*?\nconst uint8_t " + NAME + r"\[\] = \{.*?\};\n", re.S)
    if not pattern.search(src):
        raise SystemExit(f"Could not find {NAME} in {HEADER}")
    HEADER.write_text(pattern.sub(lambda m: block, src, count=1), encoding="utf-8")
    print(f"Wrote {len(data)} gzipped bytes ({HTML.stat().st_size} bytes of HTML) to {HEADER.name}")


if __name__ == "__main__":
    main()
