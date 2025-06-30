#!/usr/bin/env python3
"""
Demo script for **flo_face**
──────────────────────────

Usage
-----
$ rosrun flo_face demo_face.py happy 4

Publishes an :class:`flo_core_defs.msg.Emotion` corresponding to the first
CLI argument (``happy``, ``sad``, or ``neutral``) to the */emotion* topic
so that *face_com_manager.py* can render it.  The face stays on‑screen for
the given number of seconds (default = 3 s) and then automatically
returns to *neutral*.

Run *face_com_manager.py* in another terminal (or via a launch file)
first so it is listening on Serial.
"""

from __future__ import annotations

import sys
import time
from typing import Dict

import rospy
from flo_core_defs.msg import Emotion


EMO_MAP: Dict[str, int] = {
    "neutral": Emotion.NEUTRAL,
    "happy": Emotion.HAPPY,
    "sad": Emotion.SAD,
}

def main() -> None:
    if len(sys.argv) < 2 or sys.argv[1] in {"-h", "--help"}:
        print("Usage: demo_face.py <neutral|happy|sad> [duration_s]")
        sys.exit(0)

    face_key = sys.argv[1].lower()
    if face_key not in EMO_MAP:
        print(f"Unknown face '{face_key}'. Choices: {', '.join(EMO_MAP)}")
        sys.exit(1)

    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

    rospy.init_node("demo_face_pub", anonymous=True)
    pub = rospy.Publisher("/emotion", Emotion, queue_size=2, latch=True)

    # Small delay so that subscribers connect before we publish the latch
    rospy.sleep(0.3)

    pub.publish(Emotion(state=EMO_MAP[face_key]))
    rospy.loginfo(f"Showing '{face_key}' face for {duration:.2f} s …")
    time.sleep(duration)

    pub.publish(Emotion(state=Emotion.NEUTRAL))
    rospy.loginfo("Returned to neutral. Bye!")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
