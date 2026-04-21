#!/usr/bin/env python3
"""Inspect /dev/video* nodes and highlight likely capture devices.

Run this on the Linux host before generating camera udev rules:

    python3 utility/inspect_video_devices.py

Print only the recommended capture node:

    python3 utility/inspect_video_devices.py --recommended-only

Then use the reported node with:

    sudo python3 utility/create_udev_rules.py --camera /dev/videoX ...
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inspect /dev/video* devices and recommend a capture node."
    )
    parser.add_argument(
        "--recommended-only",
        action="store_true",
        help="Print only the recommended capture node path.",
    )
    return parser.parse_args()


def query_udev_properties(devnode: Path) -> Dict[str, str]:
    result = subprocess.run(
        ["udevadm", "info", "--query=property", f"--name={devnode}"],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        message = result.stderr.strip() or result.stdout.strip() or "unknown error"
        raise RuntimeError(f"udevadm failed for {devnode}: {message}")

    properties: Dict[str, str] = {}
    for line in result.stdout.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        properties[key] = value
    return properties


def read_sysfs_value(devnode: Path, name: str) -> str:
    path = Path("/sys/class/video4linux") / devnode.name / name
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8").strip()


def parse_device_number(devnode: Path) -> int:
    suffix = devnode.name.removeprefix("video")
    return int(suffix) if suffix.isdigit() else 10**9


def summarize_video_node(devnode: Path) -> Dict[str, str]:
    properties = query_udev_properties(devnode)
    caps = properties.get("ID_V4L_CAPABILITIES", "")
    index = read_sysfs_value(devnode, "index") or "-"
    name = read_sysfs_value(devnode, "name") or "-"
    is_capture = ":capture:" in caps

    return {
        "device": str(devnode),
        "capture": "yes" if is_capture else "no",
        "caps": caps or "-",
        "index": index,
        "name": name,
        "path": properties.get("ID_PATH", "-"),
        "serial": properties.get("ID_SERIAL_SHORT", properties.get("ID_SERIAL", "-")),
        "device_number": str(parse_device_number(devnode)),
    }


def resolve_real_path(path_str: str) -> str:
    return os.path.realpath(path_str)


def choose_recommended_node(rows: List[Dict[str, str]]) -> Optional[Dict[str, str]]:
    capture_rows = [row for row in rows if row["capture"] == "yes"]
    if not capture_rows:
        return None

    symlink_target = ""
    if Path("/dev/flo_camera").exists():
        symlink_target = resolve_real_path("/dev/flo_camera")

    for row in capture_rows:
        if resolve_real_path(row["device"]) == symlink_target:
            return row

    def sort_key(row: Dict[str, str]) -> tuple[int, int]:
        index = row["index"]
        index_value = int(index) if index.isdigit() else 10**9
        device_number = int(row["device_number"])
        return (index_value, device_number)

    return min(capture_rows, key=sort_key)


def print_table(rows: List[Dict[str, str]]) -> None:
    headers = {
        "device": "Device",
        "capture": "Capture",
        "index": "Index",
        "name": "Name",
        "caps": "V4L Caps",
    }
    columns = ["device", "capture", "index", "name", "caps"]
    widths = {
        key: max(len(headers[key]), *(len(row[key]) for row in rows))
        for key in columns
    }

    header_line = "  ".join(headers[key].ljust(widths[key]) for key in columns)
    print(header_line)
    print("  ".join("-" * widths[key] for key in columns))
    for row in rows:
        print("  ".join(row[key].ljust(widths[key]) for key in columns))


def inspect_video_nodes() -> List[Dict[str, str]]:
    video_nodes = sorted(Path("/dev").glob("video*"), key=parse_device_number)
    if not video_nodes:
        raise RuntimeError("No /dev/video* devices found.")

    rows: List[Dict[str, str]] = []
    for devnode in video_nodes:
        try:
            rows.append(summarize_video_node(devnode))
        except RuntimeError as exc:
            print(f"Warning: {exc}", file=sys.stderr)

    if not rows:
        raise RuntimeError("Unable to inspect any /dev/video* devices.")
    return rows


def main() -> int:
    args = parse_args()

    try:
        rows = inspect_video_nodes()
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    recommended = choose_recommended_node(rows)
    if args.recommended_only:
        if recommended is None:
            print("", end="")
            return 1
        print(recommended["device"])
        return 0

    print_table(rows)
    print()

    if recommended is not None:
        print("Recommended capture node:")
        print(
            f"  {recommended['device']}  "
            f"({recommended['name']}, index={recommended['index']})"
        )
        if Path("/dev/flo_camera").exists():
            current_target = resolve_real_path("/dev/flo_camera")
            print(f"Current /dev/flo_camera target: {current_target}")
    else:
        print("No capture-capable nodes detected from ID_V4L_CAPABILITIES.")
        print("Inspect the device names above and verify with a manual camera test.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
