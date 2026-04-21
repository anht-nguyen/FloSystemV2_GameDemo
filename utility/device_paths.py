#!/usr/bin/env python3
"""Resolve FLO device paths from env vars, host udev rules, or legacy defaults.

Resolution order:
1. Explicit environment variables.
2. Host udev rules file at /etc/udev/rules.d/99-flo-devices.rules.
3. Legacy kernel-assigned paths.

Useful from bash:
    eval "$(python3 utility/device_paths.py --format shell)"

Useful from Python:
    from device_paths import get_device_paths
    paths = get_device_paths()
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
from pathlib import Path
from typing import Dict

RULES_PATH = Path("/etc/udev/rules.d/99-flo-devices.rules")

LEGACY_PATHS = {
    "camera": "/dev/video0",
    "motors": "/dev/ttyUSB0",
    "face": "/dev/ttyACM0",
}

UDEV_PATHS = {
    "camera": "/dev/flo_camera",
    "motors": "/dev/flo_motors",
    "face": "/dev/flo_face",
}

ENV_MAP = {
    "camera": "FLO_CAMERA_DEVICE",
    "motors": "FLO_MOTORS_DEVICE",
    "face": "FLO_FACE_DEVICE",
}

SOURCE_ENV_MAP = {
    "camera": "FLO_CAMERA_SOURCE",
    "motors": "FLO_MOTORS_SOURCE",
    "face": "FLO_FACE_SOURCE",
}


def resolve_source_path(path_str: str) -> str:
    """Resolve a udev symlink to its backing device path without requiring the target to exist here."""
    path = Path(path_str)
    if path.is_symlink():
        target = os.readlink(path)
        if not os.path.isabs(target):
            return str((path.parent / target).resolve(strict=False))
        return str(Path(target).resolve(strict=False))
    return os.path.realpath(path_str)


def choose_source_path(preferred_path: str, legacy_path: str) -> str:
    """Choose a host source path that actually exists, falling back to legacy kernel paths."""
    resolved_preferred = resolve_source_path(preferred_path)
    if os.path.exists(resolved_preferred):
        return resolved_preferred

    resolved_legacy = resolve_source_path(legacy_path)
    if os.path.exists(resolved_legacy):
        return resolved_legacy

    # Preserve the preferred path as a final fallback so callers still see the intended stable path.
    return resolved_preferred


def get_device_paths(rules_path: Path = RULES_PATH) -> Dict[str, str]:
    """Return resolved device paths for camera, motor, and face devices."""
    paths = dict(UDEV_PATHS if rules_path.exists() else LEGACY_PATHS)

    for key, env_name in ENV_MAP.items():
        env_value = os.environ.get(env_name)
        if env_value:
            paths[key] = env_value

    for key, env_name in SOURCE_ENV_MAP.items():
        env_value = os.environ.get(env_name)
        if env_value:
            paths[f"{key}_source"] = env_value
        else:
            paths[f"{key}_source"] = choose_source_path(paths[key], LEGACY_PATHS[key])

    paths["rules_file"] = str(rules_path)
    paths["use_udev_rules"] = "1" if rules_path.exists() else "0"
    return paths


def format_shell(paths: Dict[str, str]) -> str:
    lines = []
    for key, env_name in ENV_MAP.items():
        lines.append(f"export {env_name}={shlex.quote(paths[key])}")
    for key, env_name in SOURCE_ENV_MAP.items():
        lines.append(f"export {env_name}={shlex.quote(paths[f'{key}_source'])}")
    lines.append(f"export FLO_DEVICE_RULES_FILE={shlex.quote(paths['rules_file'])}")
    lines.append(f"export FLO_USE_UDEV_RULES={shlex.quote(paths['use_udev_rules'])}")
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Resolve FLO device paths.")
    parser.add_argument(
        "--format",
        choices=("shell", "json"),
        default="shell",
        help="Output format.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    paths = get_device_paths()
    if args.format == "json":
        print(json.dumps(paths, indent=2, sort_keys=True))
    else:
        print(format_shell(paths))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
