#!/usr/bin/env python3
"""Pick a usb_cam profile from v4l2 capabilities when available.

Prints shell exports that can be eval'd before launching ROS:

    eval "$(python3 utility/select_usb_camera_profile.py)"

If v4l2-ctl is unavailable or probing fails, conservative defaults are used.
"""

from __future__ import annotations

import argparse
import os
import re
import shlex
import subprocess
from dataclasses import dataclass
from typing import Iterable, List, Optional


@dataclass(frozen=True)
class CameraMode:
    pixel_format: str
    width: int
    height: int
    fps: float


DEFAULT_DEVICE = "/dev/flo_camera"
DEFAULT_MODE = CameraMode(pixel_format="yuyv", width=640, height=480, fps=30.0)

PREFERRED_MODES: List[CameraMode] = [
    CameraMode(pixel_format="mjpeg", width=1280, height=720, fps=30.0),
    CameraMode(pixel_format="yuyv", width=640, height=480, fps=30.0),
    CameraMode(pixel_format="yuyv", width=640, height=480, fps=15.0),
    CameraMode(pixel_format="mjpeg", width=640, height=480, fps=30.0),
]

PIXEL_FORMAT_MAP = {
    "yuyv": "yuyv",
    "yuyv 4:2:2": "yuyv",
    "uyvy": "uyvy",
    "mjpeg": "mjpeg",
    "motion-jpeg": "mjpeg",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Select a supported USB camera mode.")
    parser.add_argument(
        "--device",
        default=os.environ.get("FLO_CAMERA_DEVICE", DEFAULT_DEVICE),
        help="Video device to probe.",
    )
    return parser.parse_args()


def normalize_pixel_format(value: str) -> Optional[str]:
    return PIXEL_FORMAT_MAP.get(value.strip().lower())


def parse_v4l2_modes(output: str) -> List[CameraMode]:
    modes: List[CameraMode] = []
    current_format: Optional[str] = None
    current_size: Optional[tuple[int, int]] = None

    for raw_line in output.splitlines():
        line = raw_line.strip()

        fmt_match = re.match(r"\[\d+\]: '([^']+)' \((.+)\)", line)
        if fmt_match:
            current_format = normalize_pixel_format(fmt_match.group(2)) or normalize_pixel_format(fmt_match.group(1))
            current_size = None
            continue

        size_match = re.match(r"Size: Discrete (\d+)x(\d+)", line)
        if size_match:
            current_size = (int(size_match.group(1)), int(size_match.group(2)))
            continue

        fps_match = re.match(r"Interval: Discrete [0-9.]+s \(([0-9.]+) fps\)", line)
        if fps_match and current_format and current_size:
            modes.append(
                CameraMode(
                    pixel_format=current_format,
                    width=current_size[0],
                    height=current_size[1],
                    fps=float(fps_match.group(1)),
                )
            )

    return modes


def probe_modes(device: str) -> List[CameraMode]:
    result = subprocess.run(
        ["v4l2-ctl", "--list-formats-ext", "-d", device],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        return []
    return parse_v4l2_modes(result.stdout)


def choose_mode(modes: Iterable[CameraMode]) -> CameraMode:
    mode_list = list(modes)
    if not mode_list:
        return DEFAULT_MODE

    for preferred in PREFERRED_MODES:
        for mode in mode_list:
            if (
                mode.pixel_format == preferred.pixel_format
                and mode.width == preferred.width
                and mode.height == preferred.height
                and mode.fps >= preferred.fps
            ):
                return CameraMode(
                    pixel_format=preferred.pixel_format,
                    width=preferred.width,
                    height=preferred.height,
                    fps=preferred.fps,
                )

    # Fall back to the smallest common resolution with the highest advertised fps.
    sorted_modes = sorted(
        mode_list,
        key=lambda mode: (mode.width * mode.height, -mode.fps, mode.pixel_format),
    )
    best = sorted_modes[0]
    return CameraMode(
        pixel_format=best.pixel_format,
        width=best.width,
        height=best.height,
        fps=min(best.fps, 30.0),
    )


def to_shell_exports(mode: CameraMode) -> str:
    return "\n".join(
        [
            f"export FLO_CAMERA_WIDTH={shlex.quote(str(mode.width))}",
            f"export FLO_CAMERA_HEIGHT={shlex.quote(str(mode.height))}",
            f"export FLO_CAMERA_FRAMERATE={shlex.quote(str(int(mode.fps) if mode.fps.is_integer() else mode.fps))}",
            f"export FLO_CAMERA_PIXEL_FORMAT={shlex.quote(mode.pixel_format)}",
        ]
    )


def main() -> int:
    args = parse_args()
    mode = choose_mode(probe_modes(args.device))
    print(to_shell_exports(mode))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
