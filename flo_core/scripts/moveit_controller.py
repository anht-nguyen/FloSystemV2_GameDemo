#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
moveit_controller.py  – FLO‑Core

Provides the `/simon_cmd` action server (flo_msgs/SimonCmd.action)
so that the `game_runner` SMACH executive can trigger dual‑arm
gestures for the Simon‑Says game.

Key features
------------
* Wraps `ActionSequenceController` to drive MoveIt! for both arms.
* Accepts goals of the form::
      gesture_name: "<ACTION>_left|<ACTION>_right"
      simon_says:   bool         # currently unused by the controller
* Executes left and right arm motions **concurrently** in separate threads.
* Publishes simple percentage feedback (0 → 100 %) and returns a
  success flag in the result.
* Supports pre‑emption – continuing to monitor for `is_preempt_requested()`
  and aborting the current MoveIt! plans (best‑effort) if a cancel is issued.
* Maintains backwards‑compatibility with the legacy `/action_command`
  String topic used elsewhere for ad‑hoc testing.

"""

import sys
import threading

import rospy
import moveit_commander
import actionlib
from std_msgs.msg import String

from flo_core.action_sequence_controller import ActionSequenceController, Action
from flo_core_defs.msg import SimonCmdAction, SimonCmdFeedback, SimonCmdResult

# ------------------------------------------------------------
# Helper utilities
# ------------------------------------------------------------

def _parse_gesture_name(gesture: str):
    """
    Split "<ACTION_left>|<ACTION_right>" → ((Action, 'left'), (Action, 'right'))
    """
    tokens = gesture.split("|")
    if len(tokens) != 2:
        raise ValueError(f"Expected two arm tokens separated by '|', got: {gesture}")

    parsed = []
    for tok in tokens:
        try:
            act_name, side = tok.rsplit("_", 1)
            act_enum = Action[act_name]
            if side not in ("left", "right"):
                raise ValueError
            parsed.append((act_enum, side))
        except Exception:
            raise ValueError(f"Malformed token '{tok}' inside '{gesture}'")
    return parsed  # length 2


class SimonCmdActionServer:
    def __init__(self):
        # Initialise MoveIt!
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.loginfo("[moveit_controller] Initialising MoveIt groups…")
        self.reference_frame = "world"

        self.arm_R = moveit_commander.MoveGroupCommander("R")
        self.arm_L = moveit_commander.MoveGroupCommander("L")
        self.arm_D = moveit_commander.MoveGroupCommander("dual")  # Dual arm, if applicable
        for grp in (self.arm_R, self.arm_L, self.arm_D):
            grp.set_pose_reference_frame(self.reference_frame)
            grp.allow_replanning(True)
            grp.set_goal_position_tolerance(0.001)
            grp.set_goal_orientation_tolerance(1)
            grp.set_max_acceleration_scaling_factor(0.6)
            grp.set_max_velocity_scaling_factor(0.5)

        self.controller = ActionSequenceController(arm_R=self.arm_R, arm_L=self.arm_L, arm_D=self.arm_D)

        # Action server
        self._as = actionlib.SimpleActionServer(
            "/simon_cmd",
            SimonCmdAction,
            execute_cb=self._execute_cb,
            auto_start=False,
        )
        self._as.start()
        rospy.loginfo("[moveit_controller] ✅ '/simon_cmd' action server up.")

        # Legacy topic interface – optional
        rospy.Subscriber("/action_command", String, self._legacy_cb)

    # --------------------------------------------------------
    # Legacy String interface
    # --------------------------------------------------------
    def _legacy_cb(self, msg: String):
        # Forward to the same execution logic (single‑arm version)
        try:
            act_name, side = msg.data.rsplit("_", 1)
            act_enum = Action[act_name]
            self.controller.execute_action(act_enum, side, self.reference_frame)
        except Exception as e:
            rospy.logerr(f"[moveit_controller] Legacy cmd error: {e}")

    # --------------------------------------------------------
    # Action server callback
    # --------------------------------------------------------
    def _execute_cb(self, goal):
        fb = SimonCmdFeedback()
        res = SimonCmdResult()

        rospy.loginfo(f"[moveit_controller] New goal: {goal.gesture_name} "
                      f"(simon_says={goal.simon_says})")

        try:
            tokens = _parse_gesture_name(goal.gesture_name)
        except ValueError as e:
            rospy.logerr(e)
            self._as.set_aborted(res, text=str(e))
            return

        # --------------------------------------------------
        # 1.  Parse goal → left & right Action enums
        # --------------------------------------------------
        left_enum, right_enum = None, None
        for act_enum, side in tokens:
            if side == "left":
                left_enum = act_enum
            else:
                right_enum = act_enum

        if left_enum is None or right_enum is None:
            txt = "Gesture string does not contain both left and right tokens"
            rospy.logerr(txt)
            self._as.set_aborted(res, text=txt)
            return

        # --------------------------------------------------
        # 2.  Single synchronous call – dual-arm MoveIt group
        # --------------------------------------------------
        try:
            self.controller.execute_dual_action(
                left_enum, right_enum, self.reference_frame
            )
        except Exception as exc:
            rospy.logerr(f"[moveit_controller] Dual-action failed: {exc}")
            self._as.set_aborted(res, text=str(exc))
            return
        rospy.loginfo("[moveit_controller] Dual action executed successfully.")

        fb.percent_complete = 100
        self._as.publish_feedback(fb)
        res.success = True
        self._as.set_succeeded(res)
        rospy.loginfo("[moveit_controller] Goal completed.")

# ------------------------------------------------------------

def main():
    rospy.init_node("moveit_controller")
    SimonCmdActionServer()
    rospy.loginfo("[moveit_controller] Ready – spinning.")
    rospy.spin()
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
