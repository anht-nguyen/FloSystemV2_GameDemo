#!/usr/bin/env python3
"""
FLO‑Core ─ game_runner.py (v4)
==============================
SMACH executive for a **dual‑arm** Simon‑Says game.

*Every* action in the motion library can now be assigned to *either* arm.  The
full canonical list (as defined in `action_sequence_controller.Action`) is:

    D_WAVE, D_SWING_LATERAL, D_RAISE, D_SWING_FORWARD,
    S_RAISE, S_REACH_SIDE,  S_TOUCH_HEAD, S_TOUCH_MOUTH

Defaults for both `~left_actions` and `~right_actions` parameters include **all
of them**, so the random picker has the same pool on each side.

Goal encoding remains::

    "<ACTION_NAME>_left|<ACTION_NAME>_right"

Dependencies ────────────────────────────────────────────────────────────────
* flo_msgs/PoseScore.msg
* flo_msgs/SimonCmd.action
* flo_msgs/Emotion.msg
* action_sequence_controller.Action enum
"""
from __future__ import annotations

import random
from typing import List

import rospy
import smach
import smach_ros

from flo_core_defs.msg import PoseScore, Emotion
from flo_core_defs.msg import SimonCmdAction, SimonCmdGoal

from flo_core.action_sequence_controller import Action
from flo_core.prompt_utils import build_prompt

# ────────────────────────────────────────────────────────────────────────────
# Helper states
# ────────────────────────────────────────────────────────────────────────────

def _pick_actions(pool):
    a_left = random.choice(pool)
    a_right = random.choice(pool)
    while len(pool) > 1 and a_left == a_right:
        a_right = random.choice(pool)
    return a_left, a_right

class Announce(smach.State):
    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "aborted"],
            input_keys=["left_action", "right_action", "simon_says"],
            output_keys=["left_action", "right_action", "simon_says"], 
        )

    def execute(self, ud):
        left = ud.left_action.name if ud.left_action else ""
        right = ud.right_action.name if ud.right_action else ""
        # If both are empty, log and return succeeded to avoid IndexError
        if not left and not right:
            rospy.logwarn("Announce: Both left_action and right_action are empty!")
            return "aborted"
        prompt = build_prompt(left, right, ud.simon_says)
        rospy.loginfo(prompt)
        return "succeeded"


class WaitForPose(smach.State):
    def __init__(self):
        super().__init__(
            outcomes=["matched", "timeout", "preempted"],
            input_keys=["turn_timeout"],
            output_keys=["pose_matched"],
        )
        self._latest_match = False
        self._sub = rospy.Subscriber("/pose_score", PoseScore, self._cb)

    def _cb(self, msg: PoseScore):
        if msg.matched:
            self._latest_match = True

    def execute(self, ud):
        self._latest_match = False
        start = rospy.Time.now()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self._latest_match:
                ud.pose_matched = True
                return "matched"
            if (rospy.Time.now() - start).to_sec() > ud.turn_timeout:
                ud.pose_matched = False
                return "timeout"
            if self.preempt_requested():
                self.service_preempt()
                ud.pose_matched = False
                return "preempted"
            rate.sleep()

class EvaluateState(smach.State):
    def __init__(self):
        super().__init__(
            outcomes=["good", "bad"],
            input_keys=["pose_matched", "score"],
            output_keys=["score"],
        )

    def execute(self, ud):
        if ud.pose_matched:
            ud.score += 1
            return "good"
        return "bad"

class PublishEmotionState(smach.State):
    def __init__(self, emotion_val: int):
        super().__init__(outcomes=["done"], input_keys=["face_duration"])
        self._val = emotion_val
        self._pub = rospy.Publisher("/emotion", Emotion, queue_size=1, latch=True)

    def execute(self, ud):
        self._pub.publish(Emotion(state=self._val))
        rospy.sleep(rospy.Duration(ud.face_duration))
        return "done"

class NextTurnState(smach.State):
    """Select the next random pair of actions until `total_rounds` is reached."""

    def __init__(self, action_pool: List[Action]):
        super().__init__(
            outcomes=["continue", "finished"],
            input_keys=["turn_idx", "total_rounds", "left_action", "right_action", "simon_ratio"],
            output_keys=["left_action", "right_action", "simon_says", "turn_idx"],
        )
        self._pool = action_pool

    def execute(self, ud):
        # Increment the turn index and check if the game is over
        ud.turn_idx += 1
        if ud.turn_idx > ud.total_rounds:
            return "finished"
        # Randomly select actions for both arms
        ud.left_action = random.choice(self._pool)
        ud.right_action = random.choice(self._pool)
        # Optionally ensure variety—comment out if duplicates are allowed
        # while ud.right_action == ud.left_action and len(self._pool) > 1:
        #     ud.right_action = random.choice(self._pool)

        ud.simon_says = random.random() < ud.simon_ratio
        return "continue"

# ────────────────────────────────────────────────────────────────────────────
# Build SMACH container
# ────────────────────────────────────────────────────────────────────────────

def build_sm(action_pool: List[Action], params):
    sm = smach.StateMachine(outcomes=["GAME_OVER"])

    with sm:
        # Common user‑data
        sm.userdata.turn_idx = 0
        sm.userdata.score = 0
        sm.userdata.turn_timeout = params["turn_timeout"]
        sm.userdata.total_rounds = params["total_rounds"]
        sm.userdata.simon_ratio = params["simon_ratio"]
        sm.userdata.face_duration = params["face_duration"]
        # sm.userdata.left_action = action_pool[0]
        # sm.userdata.right_action = action_pool[0]
        sm.userdata.simon_says = True
        sm.userdata.pose_matched = False
        sm.userdata.left_action, sm.userdata.right_action = _pick_actions(action_pool)
        sm.userdata.simon_says = random.random() < sm.userdata.simon_ratio

        # 1 ─ ANNOUNCE
        announce_and_cmd = smach.Concurrence(
            outcomes     = ["succeeded", "aborted", "preempted"],
            default_outcome = "aborted",
            outcome_map = {
                "succeeded": {"TALK":"succeeded", "CMD":"succeeded"},
                "aborted":   {"CMD":"aborted", "TALK":"aborted"},
                "preempted": {"CMD":"preempted"},
            },
            input_keys=["left_action", "right_action", "simon_says"],
        )
        with announce_and_cmd:
            smach.Concurrence.add("TALK", Announce())
            smach.Concurrence.add(
                "CMD",
                smach_ros.SimpleActionState(
                    "/simon_cmd",
                    SimonCmdAction,
                    goal_cb=_goal_cb,
                    input_keys=["left_action", "right_action", "simon_says"],
                    exec_timeout=rospy.Duration(15.0),
                ),
            )

        smach.StateMachine.add(
            "ANNOUNCE",
            announce_and_cmd,
            transitions={
                "succeeded": "WAIT_MOVE",
                "aborted":   "FAIL",
                "preempted": "FAIL",
            },
        )

        # 2 ─ WAIT_MOVE
        smach.StateMachine.add(
            "WAIT_MOVE",
            WaitForPose(),
            transitions={"matched": "EVALUATE", "timeout": "FAIL", "preempted": "FAIL"},
        )

        # 3 ─ EVALUATE
        smach.StateMachine.add(
            "EVALUATE",
            EvaluateState(),
            transitions={"good": "REWARD", "bad": "FAIL"},
        )

        # 4 ─ REWARD / 5 ─ FAIL
        smach.StateMachine.add(
            "REWARD",
            PublishEmotionState(Emotion.HAPPY),
            transitions={"done": "NEXT_TURN"},
        )
        smach.StateMachine.add(
            "FAIL",
            PublishEmotionState(Emotion.SAD),
            transitions={"done": "NEXT_TURN"},
        )

        # 6 ─ NEXT_TURN
        smach.StateMachine.add(
            "NEXT_TURN",
            NextTurnState(action_pool),
            transitions={"continue": "ANNOUNCE", "finished": "GAME_OVER"},
        )

    return sm

# ────────────────────────────────────────────────────────────────────────────
# Action goal callback
# ────────────────────────────────────────────────────────────────────────────

def _goal_cb(ud, _):
    goal = SimonCmdGoal()
    goal.gesture_name = f"{ud.left_action.name}_left|{ud.right_action.name}_right"
    goal.simon_says = ud.simon_says
    return goal

# ────────────────────────────────────────────────────────────────────────────
# Main
# ────────────────────────────────────────────────────────────────────────────

def main():
    rospy.init_node("game_runner")

    # Default: full action list for both arms
    default_actions = [
        "D_WAVE",
        "D_SWING_LATERAL",
        # "D_RAISE",
        # "D_SWING_FORWARD",
        "S_RAISE",
        "S_REACH_SIDE",
        "S_TOUCH_HEAD",
        "S_TOUCH_MOUTH",
    ]

    left_names = rospy.get_param("~left_actions", default_actions)
    right_names = rospy.get_param("~right_actions", default_actions)

    params = {
        "turn_timeout": rospy.get_param("~turn_timeout", 4.0),
        "face_duration": rospy.get_param("~face_duration", 1.2),
        "simon_ratio": rospy.get_param("~simon_ratio", 0.6),
        "total_rounds": rospy.get_param("~total_rounds", 15),
    }

    def to_enum(name: str):
        try:
            return Action[name]
        except KeyError:
            rospy.logwarn(f"Unknown Action '{name}' – ignored")
            return None

    # Combine left & right pools (they should be identical by default)
    pool_set = {a for a in map(to_enum, left_names + right_names) if a}
    action_pool = list(pool_set)

    assert action_pool, "Action list must not be empty"

    sm = build_sm(action_pool, params)

    sis = smach_ros.IntrospectionServer("game_sm", sm, "/GAME_SM")
    sis.start()
    rospy.loginfo("[game_runner] Dual‑arm Simon Says started")
    outcome = sm.execute()
    rospy.loginfo("[game_runner] Finished with outcome: %s", outcome)
    sis.stop()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
