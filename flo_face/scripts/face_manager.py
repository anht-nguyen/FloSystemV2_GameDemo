#!/usr/bin/env python3
"""
flo_face : face_manager.py
──────────────────────────
• Subscribes to /emotion (flo_core_defs/Emotion)
• Sends a single-byte code to the Teensy via `serial_coms.SerialCom`
      0 = NEUTRAL   1 = HAPPY   2 = SAD
• Reconnects automatically if the USB cable is yanked.
• Optional auto-reset back to NEUTRAL after `~neutral_timeout` seconds.
"""

import threading
import rospy
from serial import SerialException
from serial_coms import SerialCom            # same helper used in Lil’Flo
from flo_core_defs.msg import Emotion

class FaceComManager:
    def __init__(self):
        rospy.init_node("face_com_manager")

        # ── Parameters ──────────────────────────────────────────────
        self.port            = rospy.get_param("~port", "/dev/flo_face")
        self.neutral_timeout = float(rospy.get_param("~face_duration"))

        # ── Serial connection (w/ auto-reconnect) ──────────────────
        self.coms = None
        self._connect_loop()

        # ── Internal state ─────────────────────────────────────────
        self._last_state  = None          # suppress redundant writes
        self._reset_timer = None
        self._lock        = threading.Lock()

        # ── ROS subscriber ─────────────────────────────────────────
        rospy.Subscriber("/emotion", Emotion, self._emotion_cb, queue_size=5)
        rospy.loginfo("[face_com_manager] Ready – listening on /emotion")
        rospy.spin()

    # ────────────────────────────────────────────────────────────────
    # Serial handling
    # ────────────────────────────────────────────────────────────────
    def _connect_loop(self):
        """Try to open SerialCom every 2 s until it succeeds or ROS shuts down."""
        rate = rospy.Rate(2)                              # 2 Hz retry
        while not self.coms and not rospy.is_shutdown():
            try:
                self.coms = SerialCom(self.port,
                                       self._data_handler,
                                       write_timeout=1)
                rospy.loginfo(f"[face_com_manager] Connected on {self.port}")
            except SerialException as exc:
                rospy.logdebug_throttle(10.0,
                    f"[face_com_manager] Serial open failed: {exc}")
                self.coms = None
                rate.sleep()

    def _send_state(self, value: int):
        """Thread-safe helper that writes one byte via SerialCom."""
        if not self.coms:                    # lost connection → try again
            self._connect_loop()
        if self.coms:
            try:
                with self._lock:
                    self.coms.sendData([value & 0xFF])
            except SerialException as exc:
                rospy.logerr(f"[face_com_manager] Serial write failed: {exc}")
                self.coms = None             # force reconnect next call

    @staticmethod
    def _data_handler(*data):
        """Print anything the Teensy spits back (debug only)."""
        rospy.logdebug(f"[face_com_manager] RX: {data}")

    # ────────────────────────────────────────────────────────────────
    # ROS callback
    # ────────────────────────────────────────────────────────────────
    def _emotion_cb(self, msg: Emotion):
        """
        • Writes the emotion state to the Teensy via SerialCom.
        • Optionally schedules a timed return to NEUTRAL.
        """
        if msg.state == self._last_state and self._reset_timer is None:
            return

        self._send_state(msg.state)
        self._last_state = msg.state

        # schedule automatic NEUTRAL reset
        if self.neutral_timeout > 0 and msg.state != Emotion.NEUTRAL:
            if self._reset_timer:
                self._reset_timer.cancel()
            self._reset_timer = threading.Timer(
                self.neutral_timeout, self._reset_face)
            self._reset_timer.start()

    def _reset_face(self):
        self._send_state(Emotion.NEUTRAL)
        self._last_state  = Emotion.NEUTRAL
        self._reset_timer = None


# ────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    try:
        FaceComManager()
    except rospy.ROSInterruptException:
        pass
