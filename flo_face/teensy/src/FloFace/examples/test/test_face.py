#!/usr/bin/env python3
"""Send Lil'Flo face expressions directly to the Teensy over serial.

Examples
--------
Show one face:
    python3 test_face.py --port /dev/flo_face happy

Cycle through a few faces:
    python3 test_face.py --port /dev/flo_face neutral happy sad surprised

List the available face names from faces.json:
    python3 test_face.py --list
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Iterable


THIS_FILE = Path(__file__).resolve()


def find_flo_face_root(start: Path) -> Path:
    """Walk upward until we find the repository's flo_face directory."""
    for directory in (start.parent, *start.parents):
        if directory.name == "flo_face" and (directory / "data" / "faces.json").exists():
            return directory
        candidate = directory / "flo_face"
        if (candidate / "data" / "faces.json").exists():
            return candidate
    raise FileNotFoundError("Could not locate flo_face/data/faces.json from this script")


FLO_FACE_ROOT = find_flo_face_root(THIS_FILE)
REPO_ROOT = FLO_FACE_ROOT.parent
DEFAULT_FACES_JSON = FLO_FACE_ROOT / "data" / "faces.json"

for candidate in (
    FLO_FACE_ROOT / "scripts",
    FLO_FACE_ROOT / "teensy" / "src" / "serial_coms" / "computer" / "python" / "serial-coms",
):
    candidate_str = str(candidate)
    if candidate_str not in sys.path:
        sys.path.insert(0, candidate_str)

from serial import SerialException
from serial_coms import SerialCom


def flatten(grid: list[list[int]]) -> list[int]:
    """Flatten a 2-D matrix in row-major order."""
    return [item for row in grid for item in row]


def bytize(bits: Iterable[int]) -> list[int]:
    """Pack 0/1 values into bytes, most-significant-bit first."""
    bit_list = list(bits)
    packed = [0] * math.ceil(len(bit_list) / 8)
    for index, value in enumerate(bit_list):
        if value:
            packed[index // 8] |= 1 << (7 - (index % 8))
    return packed


class FaceSerialTester:
    """Load face definitions and ship them to the Teensy protocol."""

    def __init__(self, port: str, faces_path: Path, brightness: int):
        self.port = port
        self.faces_path = faces_path
        self.brightness = max(0, min(15, brightness))
        self.face_data = self._load_faces()
        self.current_eye_dir = "center"
        self.coms: SerialCom | None = None

    def _load_faces(self) -> dict:
        with self.faces_path.open("r", encoding="utf-8") as faces_file:
            return json.load(faces_file)

    @staticmethod
    def _data_handler(*data):
        if data:
            try:
                message = bytes(data).decode("ascii", errors="replace").rstrip("\x00")
            except ValueError:
                message = str(data)
            print(f"[teensy] {message}")

    def available_faces(self) -> list[str]:
        return sorted(
            name for name in self.face_data["mouths"] if not name.startswith("_")
        )

    def compose_bits(self, face_name: str) -> tuple[list[int], list[int], list[int]]:
        mouths = self.face_data["mouths"]
        eyes = self.face_data["eyes"]

        if face_name not in mouths:
            raise KeyError(f"Unknown face '{face_name}'")

        mouth_entry = mouths[face_name]
        mouth_bits = flatten(mouth_entry["on"])

        eye_set_name = mouth_entry["eyes"]
        eye_set = eyes[eye_set_name]
        if self.current_eye_dir not in eye_set:
            self.current_eye_dir = eye_set.get(
                "default", next(key for key in eye_set if key != "default")
            )

        eye_block = eye_set[self.current_eye_dir]
        while isinstance(eye_block, str):
            self.current_eye_dir = eye_block
            eye_block = eye_set[self.current_eye_dir]

        if "on" in eye_block:
            left_eye_bits = flatten(eye_block["on"])
            right_eye_bits = left_eye_bits
        else:
            left_eye_bits = flatten(eye_block["left"]["on"])
            right_eye_bits = flatten(eye_block["right"]["on"])

        return mouth_bits, left_eye_bits, right_eye_bits

    def send_section(self, command_id: int, bits: list[int]) -> None:
        self._ensure_connected()
        self.coms.sendData([command_id] + bytize(bits))
        time.sleep(0.05)
        self.coms.receiveData(iterations=200)

    def set_brightness(self) -> None:
        self._ensure_connected()
        for command_id in (3, 4, 5):
            self.coms.sendData([command_id, self.brightness])
            time.sleep(0.05)
            self.coms.receiveData(iterations=200)

    def _ensure_connected(self) -> None:
        if self.coms is None:
            self.coms = SerialCom(self.port, self._data_handler, write_timeout=1)

    def show_face(self, face_name: str) -> None:
        mouth_bits, left_eye_bits, right_eye_bits = self.compose_bits(face_name)
        print(f"Displaying '{face_name}' on {self.port}")
        self.send_section(0, mouth_bits)
        self.send_section(1, left_eye_bits)
        self.send_section(2, right_eye_bits)
        self.set_brightness()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Display named faces on the Lil'Flo Teensy face hardware."
    )
    parser.add_argument(
        "faces",
        nargs="*",
        default=["neutral", "happy", "sad"],
        help="One or more face names to display. Defaults to a small demo cycle.",
    )
    parser.add_argument(
        "--port",
        default="/dev/flo_face",
        help="Serial device for the Teensy face controller.",
    )
    parser.add_argument(
        "--faces-json",
        type=Path,
        default=DEFAULT_FACES_JSON,
        help="Path to the faces.json file to load.",
    )
    parser.add_argument(
        "--brightness",
        type=int,
        default=12,
        help="Display brightness from 0 to 15.",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=1.5,
        help="Seconds to wait between faces when cycling.",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Repeat the provided face sequence until interrupted.",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="Print available face names and exit.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    tester = FaceSerialTester(args.port, args.faces_json, args.brightness)

    if args.list:
        print("\n".join(tester.available_faces()))
        return 0

    for face_name in args.faces:
        if face_name not in tester.face_data["mouths"]:
            print(
                f"Unknown face '{face_name}'. Use --list to see valid face names.",
                file=sys.stderr,
            )
            return 1

    try:
        while True:
            for face_name in args.faces:
                tester.show_face(face_name)
                time.sleep(args.delay)
            if not args.loop:
                break
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    except SerialException as exc:
        print(f"Serial error on {args.port}: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
