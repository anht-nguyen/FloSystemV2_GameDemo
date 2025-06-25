#!/usr/bin/env python3
"""
FLO‑Core ─ game_runner.py
========================
Executive state‑machine that coordinates a responsive “Simon Says” game.

*   Publishes/consumes only high‑level ROS APIs – it does **not** touch
    low‑level hardware or vision code directly.
*   Built with **SMACH** so that behaviour can be visualised live in
    *smach_viewer* and unit‑tested in isolation.
*   Compatible with ROS Noetic (Python 3).

Topics & actions
----------------
/pose_score            (flo_msgs/PoseScore)        – vision result
events
/simon_cmd             (flo_msgs/SimonCmdAction)   – robot gesture
/emotion               (flo_msgs/Emotion)          – face display

Custom userdata keys
--------------------
gesture_name : str     – YAML key for next gesture
simon_says   : bool    – whether the prompt includes “Simon says”
turn_idx     : int     – running counter for statistics
score        : int     – accumulated player score
turn_timeout : float   – seconds allowed to respond each turn
pose_matched : bool    – result latched from vision layer
"""

from __future__ import annotations

import random
from typing import List, Dict

import rospy
import smach
import smach_ros
from std_msgs.msg import Header

# FLO custom interfaces
from flo_core_defs.msg import PoseScore, Emotion
from flo_core_defs.msg import SimonCmdAction, SimonCmdGoal


# ---------------------------------------------------------------------------
#  Simple helper states
# ---------------------------------------------------------------------------
class WaitForPose(smach.State):
    """State that waits until the player matches the target pose *or* times out."""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["matched", "timeout", "preempted"],
            input_keys=["turn_timeout"],
            output_keys=["pose_matched"],
        )
        self._pose_sub = rospy.Subscriber("/pose_score", PoseScore, self._pose_cb)
        self._latest_match = False

    # --------------------------------------------------
    def _pose_cb(self, msg: PoseScore):  # noqa: D401
        """Latched callback; only considers *True* events."""
        if msg.matched:
            self._latest_match = True

    # --------------------------------------------------
    def execute(self, userdata):  # noqa: D401
        self._latest_match = False
        start = rospy.Time.now()
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self._latest_match:
                userdata.pose_matched = True
                return "matched"
            if (rospy.Time.now() - start).to_sec() > userdata.turn_timeout:
                userdata.pose_matched = False
                return "timeout"
            if self.preempt_requested():
                self.service_preempt()
                userdata.pose_matched = False
                return "preempted"
            rate.sleep()


class EvaluateState(smach.State):
    """Assign *good* or *bad* outcome based on WaitForPose result."""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["good", "bad"],
            input_keys=["pose_matched", "score"],
            output_keys=["score"],
        )

    def execute(self, userdata):  # noqa: D401
        if userdata.pose_matched:
            userdata.score += 1
            return "good"
        return "bad"


class PublishEmotionState(smach.State):
    """Publish an Emotion message and pause briefly so the face can animate."""

    def __init__(self, emotion_val: int, duration: float = 1.5):
        super().__init__(outcomes=["done"])
        self._emotion_val = emotion_val
        self._duration = rospy.Duration(duration)
        self._pub = rospy.Publisher("/emotion", Emotion, queue_size=1, latch=True)

    def execute(self, _):  # noqa: D401
        self._pub.publish(Emotion(state=self._emotion_val))
        rospy.sleep(self._duration)
        return "done"


class NextTurnState(smach.State):
    """Pick the next gesture and set *Simon says* flag randomly."""

    def __init__(self, gestures: List[str]):
        smach.State.__init__(
            self,
            outcomes=["continue", "finished"],
            input_keys=["turn_idx"],
            output_keys=[
                "gesture_name",
                "simon_says",
                "turn_idx",
                "pose_matched",
            ],
        )
        self._gestures = gestures

    def execute(self, userdata):  # noqa: D401
        userdata.turn_idx += 1
        if userdata.turn_idx >= len(self._gestures):
            return "finished"

        next_gesture = self._gestures[userdata.turn_idx]
        userdata.gesture_name = next_gesture
        userdata.simon_says = bool(random.getrandbits(1))
        userdata.pose_matched = False  # reset flag
        return "continue"


# ---------------------------------------------------------------------------
#  Build the top-level SMACH container
# ---------------------------------------------------------------------------

def build_game_state_machine(gestures: List[str], turn_timeout: float = 10.0):
    sm = smach.StateMachine(outcomes=["GAME_OVER"])

    # Set initial user‑data
    sm.userdata.gesture_name = gestures[0]
    sm.userdata.simon_says = True
    sm.userdata.turn_idx = 0
    sm.userdata.score = 0
    sm.userdata.turn_timeout = turn_timeout
    sm.userdata.pose_matched = False

    with sm:
        # 1 ─ ANNOUNCE (robot action)
        smach.StateMachine.add(
            "ANNOUNCE",
            smach_ros.SimpleActionState(
                "/simon_cmd",
                SimonCmdAction,
                goal_cb=_goal_cb,
                input_keys=["gesture_name", "simon_says"],
                output_keys=["executed"],
                exec_timeout=rospy.Duration(5.0),
                preempt_timeout=rospy.Duration(1.0),
            ),
            transitions={
                "succeeded": "WAIT_MOVE", 
                "aborted": "FAIL",
                "preempted": "FAIL"},
        )

        # 2 ─ WAIT_MOVE (player imitates)
        smach.StateMachine.add(
            "WAIT_MOVE",
            WaitForPose(),
            transitions={
                "matched": "EVALUATE",
                "timeout": "FAIL",
                "preempted": "FAIL",
            },
        )

        # 3 ─ EVALUATE (compute score)
        smach.StateMachine.add(
            "EVALUATE",
            EvaluateState(),
            transitions={"good": "REWARD", "bad": "FAIL"},
        )

        # 4 ─ REWARD
        smach.StateMachine.add(
            "REWARD",
            PublishEmotionState(Emotion.HAPPY),
            transitions={"done": "NEXT_TURN"},
        )

        # 5 ─ FAIL
        smach.StateMachine.add(
            "FAIL",
            PublishEmotionState(Emotion.SAD),
            transitions={"done": "NEXT_TURN"},
        )

        # 6 ─ NEXT_TURN
        smach.StateMachine.add(
            "NEXT_TURN",
            NextTurnState(gestures),
            transitions={"continue": "ANNOUNCE", "finished": "GAME_OVER"},
        )

    return sm


# ---------------------------------------------------------------------------
#  SimpleActionState goal callback
# ---------------------------------------------------------------------------

def _goal_cb(userdata, goal: SimonCmdGoal):  # noqa: D401
    goal = SimonCmdGoal()
    goal.gesture_name = userdata.gesture_name
    goal.simon_says = userdata.simon_says
    return goal


# ---------------------------------------------------------------------------
#  Main entry point
# ---------------------------------------------------------------------------


def main() -> None:  # noqa: D401
    rospy.init_node("game_runner")

    # Load gesture list from parameter or fallback to defaults
    gestures: List[str] = rospy.get_param(
        "~gestures", [
            "wave", "arms_up", "dab", "hands_on_head", "swipe_left", "swipe_right"
        ])
    timeout_sec: float = rospy.get_param("~turn_timeout", 10.0)

    sm = build_game_state_machine(gestures, timeout_sec)

    # Introspection server wires the machine to smach_viewer
    sis = smach_ros.IntrospectionServer("game_sm", sm, "/GAME_SM")
    sis.start()

    rospy.loginfo("[game_runner] ▶▶ Starting Simon Says SMACH executive")

    outcome = sm.execute()
    rospy.loginfo("[game_runner] Finished with outcome: %s", outcome)

    sis.stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
