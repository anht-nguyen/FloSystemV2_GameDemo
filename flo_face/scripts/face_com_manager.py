#!/usr/bin/env python3
"""
flo_face : face_com_manager.py
──────────────────────────────
ROS→Teensy bridge that renders Lil'Flo LED‑matrix faces based on the
`/emotion` topic. Instead of sending a one‑byte emoji code, this version
reads the full pixel definitions from **faces.json** so new expressions can
be dropped in without recompiling firmware or touching Python.

Expected Teensy protocol (simple and robust):
    • One frame → binary blob of RGB bytes (0‑255) in R‑G‑B order.
      The Teensy must already know how to map the incoming pixel stream
      to its LED matrix (e.g. serpentine 16×16).
    • Host blocks until the whole frame is acknowledged (optional ACK).

If your firmware requires a header/command byte, add it in
`_send_pixels()` – the rest of the code stays unchanged.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import List

import rospy
import rospkg
from serial import SerialException
from std_msgs.msg import String

from serial_coms import SerialCom  # same helper used elsewhere in FloSystem
from flo_core_defs.msg import Emotion


class FaceComManager:
    """Streams pixel frames to the Teensy according to incoming *Emotion* messages."""

    # ────────────────────────────────────────────────────────────────
    # Initialisation
    # ────────────────────────────────────────────────────────────────
    def __init__(self):
        rospy.init_node("face_com_manager")

        # Parameters -------------------------------------------------
        self.port: str = rospy.get_param("~port", "/dev/flo_face")
        self.neutral_timeout: float = float(rospy.get_param("~face_duration", 3.0))
        self.brightness: int = int(rospy.get_param("~brightness", 12))  # 0‑15 matches old API

        # Faces ------------------------------------------------------
        self.face_data = self._load_faces()
        self.emotion_to_mouth = {
            Emotion.NEUTRAL: "neutral",
            Emotion.HAPPY: "happy",
            Emotion.SAD: "sad",
        }
        self.current_eye_set: str = "standard"
        self.current_eye_dir: str = "center"

        # Serial -----------------------------------------------------
        self._lock = threading.Lock()
        self.coms: SerialCom | None = None
        self._connect_loop()

        # Timed auto‑reset ------------------------------------------
        self._last_state: int | None = None
        self._last_face_key: str | None = None
        self._conversation_face_active = False
        self._reset_timer: threading.Timer | None = None

        # ROS subscription ------------------------------------------
        rospy.Subscriber("/emotion", Emotion, self._emotion_cb, queue_size=5)
        rospy.Subscriber("/conversation/face", String, self._face_name_cb, queue_size=5)
        rospy.Subscriber("/conversation/state", String, self._conversation_state_cb, queue_size=5)
        rospy.loginfo("[face_com_manager] Ready – listening on /emotion")
        rospy.spin()

    # ────────────────────────────────────────────────────────────────
    # Face loading helpers
    # ────────────────────────────────────────────────────────────────
    def _load_faces(self):
        rospack = rospkg.RosPack()
        faces_json = rospy.get_param(
            "~faces_json",
            os.path.join(rospack.get_path("flo_face"), "data", "faces.json"),
        )
        try:
            with open(faces_json, "r", encoding="utf-8") as fp:
                data = json.load(fp)
            rospy.loginfo(
                f"[face_com_manager] Loaded {len(data['mouths'])} mouths and "
                f"{len(data['eyes'])} eye-sets from {faces_json}"
            )
            return data
        except (OSError, json.JSONDecodeError) as exc:
            rospy.logerr(f"[face_com_manager] Cannot read {faces_json}: {exc}")
            rospy.signal_shutdown("faces.json missing or malformed")
            raise  # never reached, but helps static checkers

    # Utilities -----------------------------------------------------
    @staticmethod
    def _flatten(grid: List[List[int]]) -> List[int]:
        """Flatten a 2‑D matrix (row‑major) into a 1‑D list."""
        return [item for row in grid for item in row]

    # ────────────────────────────────────────────────────────────────
    # Serial handling
    # ────────────────────────────────────────────────────────────────
    def _connect_loop(self):
        """Try to open SerialCom every 2 s until it succeeds or ROS shuts down."""
        rate = rospy.Rate(2)  # 2 Hz retry
        while not self.coms and not rospy.is_shutdown():
            try:
                self.coms = SerialCom(self.port, self._data_handler, write_timeout=1)
                rospy.loginfo(f"[face_com_manager] Connected on {self.port}")
            except SerialException as exc:
                rospy.logwarn_throttle(
                    10.0, f"[face_com_manager] Serial open failed: {exc} – retrying"
                )
                self.coms = None
                rate.sleep()

    def _send_pixels(self, pixels: List[int]):
        """Thread‑safe wrapper that ships one full RGB frame to the Teensy."""
        if not self.coms:
            self._connect_loop()
        if not self.coms:
            return  # cannot recover – give up for now
        try:
            with self._lock:
                self.coms.sendData(pixels)
        except SerialException as exc:
            rospy.logerr(f"[face_com_manager] Serial write failed: {exc}")
            self.coms = None  # force reconnect next time

    @staticmethod
    def _data_handler(*data):
        """Dump anything we receive back from the Teensy (debug only)."""
        rospy.logdebug(f"[face_com_manager] RX: {data}")

    # ────────────────────────────────────────────────────────────────
    # Helpers for legacy Teensy serial protocol
    # ────────────────────────────────────────────────────────────────
    @staticmethod
    def _bytize(bits):
        """Pack a list/tuple of 0-1 ints into bytes (MSB-first, 8 pixels / byte)."""
        if not isinstance(bits, (list, tuple)):
            bits = [bits]
        out_len = math.ceil(len(bits) / 8)
        packed = [0] * out_len
        for i in range(out_len):
            byte = 0
            for j in range(8):
                idx = i * 8 + j
                if idx < len(bits) and bits[idx]:
                    byte |= 1 << (7 - j)
            packed[i] = byte
        return packed

    def _send_section(self, cmd_id: int, bits):
        """
        Teensy expects one command byte (0-5) followed by packed data.
        0 = mouth, 1 = R-eye, 2 = L-eye, 3-5 = brightness / misc.
        """
        self._send_pixels([cmd_id] + self._bytize(bits))

    # ────────────────────────────────────────────────────────────────
    # Build monochrome bit-maps for the old firmware
    def _compose_bits(self, mouth_key: str):
        """Return (mouth_bits, left_bits, right_bits) – each a flat 0/1 list."""
        mouths = self.face_data["mouths"]
        eyes   = self.face_data["eyes"]

        if mouth_key not in mouths:
            rospy.logwarn_once(f"[face_com_manager] Unknown mouth '{mouth_key}'")
            mouth_key = "neutral"

        mouth_on           = mouths[mouth_key]["on"]
        self.current_eye_set = mouths[mouth_key]["eyes"]

        # ── pick a valid gaze direction ───────────────────────────
        eye_cfg = eyes[self.current_eye_set]            # whole set (“standard”, …)
        if self.current_eye_dir not in eye_cfg:
            # Follow the JSON pointer – often "default": "center"
            self.current_eye_dir = eye_cfg.get(
                "default", next(k for k in eye_cfg if k != "default")
            )

        eye_block = eye_cfg[self.current_eye_dir]        # guaranteed dict *or* pointer
        # The block itself can still be a string if there are multiple indirections
        while isinstance(eye_block, str):
            self.current_eye_dir = eye_block
            eye_block = eye_cfg[self.current_eye_dir]

        # ── extract bit-maps ──────────────────────────────────────
        if "on" in eye_block:                            # shared matrix for both eyes
            left_on = right_on = eye_block["on"]
        else:                                            # separate `"left"` / `"right"`
            left_on  = eye_block["left"]["on"]
            right_on = eye_block["right"]["on"]

        return (
            self._flatten(mouth_on),
            self._flatten(left_on),
            self._flatten(right_on),
        )

    # ────────────────────────────────────────────────────────────────
    # Face composition
    # ────────────────────────────────────────────────────────────────
    def _compose_frame(self, mouth_key: str) -> List[int]:
        """Build a full RGB frame from faces.json ready to stream via serial."""
        mouths = self.face_data["mouths"]
        eyes = self.face_data["eyes"]

        if mouth_key not in mouths:
            rospy.logwarn_once(f"[face_com_manager] Unknown mouth '{mouth_key}'.")
            mouth_key = "neutral"

        mouth = mouths[mouth_key]["on"]
        self.current_eye_set = mouths[mouth_key]["eyes"]

        eye_conf = eyes[self.current_eye_set]
        if self.current_eye_dir not in eye_conf:
            self.current_eye_dir = eye_conf["default"]

        eye_data = eye_conf[self.current_eye_dir]

        # left/right may be separated; fall back to shared matrix
        if "left" in eye_data:
            left_eye = eye_data["left"]["on"]
            right_eye = eye_data["right"]["on"]
        else:
            left_eye = right_eye = eye_data["on"]

        # Concat in fixed order: leftEye → rightEye → mouth
        frame = self._flatten(left_eye) + self._flatten(right_eye) + self._flatten(mouth)

        # Optional brightness scaling (faces.json RGB values are 0‑15)
        if self.brightness != 15:  # 15 == max in original schema
            scale = self.brightness / 15.0
            frame = [min(255, int(c * scale)) for c in frame]

        return frame

    # ────────────────────────────────────────────────────────────────
    # ROS callback
    # ────────────────────────────────────────────────────────────────
    def _show_face(self, mouth_key: str):
        mouth_b, left_b, right_b = self._compose_bits(mouth_key)
        self._send_section(0, mouth_b)
        self._send_section(1, right_b)
        self._send_section(2, left_b)
        self._send_pixels([3, int(self.brightness & 0x0F)])
        self._last_face_key = mouth_key

        if self.neutral_timeout > 0 and mouth_key != "neutral":
            if self._reset_timer:
                self._reset_timer.cancel()
            self._reset_timer = threading.Timer(self.neutral_timeout, self._reset_neutral)
            self._reset_timer.start()

    def _emotion_cb(self, msg: Emotion):
        """Render a face matching the *Emotion* enum and schedule auto‑reset."""
        if msg.state == self._last_state and self._reset_timer is None:
            return  # nothing new

        mouth = self.emotion_to_mouth.get(msg.state, "neutral")
        self._show_face(mouth)
        self._last_state = msg.state

    def _face_name_cb(self, msg: String):
        mouth_key = msg.data.strip()
        if not mouth_key:
            return
        if mouth_key not in self.face_data["mouths"]:
            rospy.logwarn_throttle(10.0, f"[face_com_manager] Unknown conversation face '{mouth_key}'")
            mouth_key = "neutral"
        if mouth_key == self._last_face_key and self._reset_timer is None:
            return

        self._conversation_face_active = True
        self._show_face(mouth_key)
        self._last_state = None

    def _conversation_state_cb(self, msg: String):
        state = msg.data.strip().lower()
        if state == "speaking":
            self._conversation_face_active = True
            return
        if state == "idle" and self._conversation_face_active:
            if self._reset_timer:
                self._reset_timer.cancel()
                self._reset_timer = None
            self._conversation_face_active = False
            self._reset_neutral()

    def _reset_neutral(self):
        self._show_face("neutral")
        self._last_state = Emotion.NEUTRAL
        self._conversation_face_active = False
        self._reset_timer = None


# ────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    try:
        FaceComManager()
    except rospy.ROSInterruptException:
        pass
