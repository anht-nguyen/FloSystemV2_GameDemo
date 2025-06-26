# action_sequence_controller.py
import rospy
from enum import IntEnum

class Action(IntEnum):
    HOME               = 0
    # Dynamic (repeated) actions
    D_WAVE             = 10
    D_SWING_LATERAL    = 11
    D_RAISE            = 12 # Note: might not be used due to potential limitations with CV detection
    D_SWING_FORWARD    = 13 # Note: might not be used due to potential limitations with CV detection
    # Static (single) actions
    S_RAISE            = 20
    S_REACH_SIDE       = 21
    S_TOUCH_HEAD       = 22
    S_TOUCH_MOUTH      = 23
    # … extend with more actions as needed …

class ActionSequenceController:
    def __init__(self, arm_R, arm_L, arm_D=None):
        """
        arm_R: MoveGroupCommander for right arm
        arm_L: MoveGroupCommander for left arm
        arm_D: MoveGroupCommander for dual arm (if applicable)
        """
        self.arm_R = arm_R
        self.arm_L = arm_L
        self.arm_D = arm_D

        # Map action types to handler methods
        self._action_map = {
            Action.HOME:            self._go_home,
            Action.D_WAVE:          self._do_wave,
            Action.D_RAISE:         self._do_raise_dynamic,
            Action.D_SWING_LATERAL: self._do_swing_lateral,
            Action.D_SWING_FORWARD: self._do_swing_forward,
            Action.S_RAISE:         self._do_raise_static,
            Action.S_REACH_SIDE:    self._do_reach_side,
        }

    def execute_action(self, action: Action, side: str, reference_frame: str = 'world') -> None:
        """
        Execute the specified Action with the given arm side ('right' or 'left').
        Moves to home, performs the action, then returns home.
        """
        rospy.loginfo(f"Action {action.name} on {side}-arm in frame '{reference_frame}'")
        arm = self._select_arm(side)
        self._go_home()
        handler = self._action_map.get(action, self._unknown_action)
        handler(arm)
        self._go_home()

    def _select_arm(self, side: str):
        """Return the MoveGroupCommander for the requested side."""
        if side.lower().startswith('r'):
            return self.arm_R
        elif side.lower().startswith('l'):
            return self.arm_L
        else:
            rospy.logwarn(f"Unknown side '{side}', defaulting to right arm")
            return self.arm_R

    def _go_home(self, arm=None) -> None:
        """Send both arms to their home positions."""
        # Home both arms to their named targets
        self.arm_R.set_named_target('Rhome')
        self.arm_R.go()
        self.arm_L.set_named_target('Lhome')
        self.arm_L.go()

    # Dynamic actions (repeat 3 times)
    def _do_wave(self, arm) -> None:
        for _ in range(3):
            arm.set_named_target(f"{arm.get_name()}_wave_start")
            arm.go()
            arm.set_named_target(f"{arm.get_name()}_wave_end")
            arm.go()

    def _do_raise_dynamic(self, arm) -> None:
        for _ in range(3):
            arm.set_named_target(f"{arm.get_name()}_raise")
            arm.go()
            arm.set_named_target(f"{arm.get_name()}_home")
            arm.go()

    def _do_swing_lateral(self, arm) -> None:
        for _ in range(3):
            arm.set_named_target(f"{arm.get_name()}_waveb")
            arm.go()
            arm.set_named_target(f"{arm.get_name()}_d_bell")
            arm.go()

    def _do_swing_forward(self, arm) -> None:
        # placeholder: forward/backward swing implementation
        for _ in range(3):
            arm.set_named_target(f"{arm.get_name()}_swing_fwd")
            arm.go()
            arm.set_named_target(f"{arm.get_name()}_swing_bwd")
            arm.go()

    # Static actions (single execution)
    def _do_raise_static(self, arm) -> None:
        arm.set_named_target(f"{arm.get_name()}_raise")
        arm.go()

    def _do_reach_side(self, arm) -> None:
        arm.set_named_target(f"{arm.get_name()}_reach_side")
        arm.go()

    def _unknown_action(self, arm=None) -> None:
        rospy.logerr("Unknown action requested; no movement executed.")


# --------------------------------------------------
    # Dual-arm version
    # --------------------------------------------------
    def execute_dual_action(
        self,
        left_action: Action,
        right_action: Action,
        reference_frame: str = "world",
    ) -> None:
        """
        Execute <left_action> on the left arm **simultaneously** with
        <right_action> on the right arm, by sending a single trajectory
        to the dual-arm MoveIt group.  Falls back to sequential execution
        if no dual group was supplied.
        """
        if self.arm_D is None:
            # Fallback – old behaviour (sequential two-thread)
            self.execute_action(left_action, "left", reference_frame)
            self.execute_action(right_action, "right", reference_frame)
            return

        rospy.loginfo(
            f"Dual action {left_action.name} (L) | {right_action.name} (R)"
        )

        self._go_home_dual()

        # For now reuse the *single-arm* handlers but merge the named-target
        # joint dictionaries into a single goal for arm_D.  This keeps the
        # amount of extra code tiny while guaranteeing simultaneity.
        self._run_dual_step(left_action, right_action)

        self._go_home_dual()

    # ---------- helpers ----------
    def _run_dual_step(self, left_action: Action, right_action: Action):
        """
        Send the first pose of left_action and right_action together,
        then, if they are dynamic, repeat their internal cycle 2× more.
        Uses the existing private single-arm handlers to get named-target
        strings, merges the joint-value dicts, and calls arm_D.go().
        """
        # Step generator for each arm -------------------
        def _pose_seq(action_enum, arm_name):
            if action_enum == Action.D_WAVE:
                return [f"{arm_name}_wave_start", f"{arm_name}_wave_end"] * 3
            elif action_enum == Action.D_SWING_LATERAL:
                return [f"{arm_name}_waveb", f"{arm_name}_d_bell"] * 3
            elif action_enum == Action.S_RAISE:
                return [f"{arm_name}_raise"]
            elif action_enum == Action.S_REACH_SIDE:
                return [f"{arm_name}_reach_side"]
            elif action_enum == Action.D_RAISE:
                return [f"{arm_name}_raise", f"{arm_name}_home"] * 3
            elif action_enum == Action.D_SWING_FORWARD:
                return [f"{arm_name}_swing_fwd", f"{arm_name}_swing_bwd"] * 3
            else:  # HOME or unknown → empty
                return []

        seq_L = _pose_seq(left_action,  "L")
        seq_R = _pose_seq(right_action, "R")

        # Execute the longest of the two sequences; if one arm finishes early
        # it just holds its last pose.
        for idx in range(max(len(seq_L), len(seq_R))):
            tgt = {}
            if idx < len(seq_L):
                tgt.update(self.arm_L.get_named_target_values(seq_L[idx]))
            if idx < len(seq_R):
                tgt.update(self.arm_R.get_named_target_values(seq_R[idx]))
            self.arm_D.set_joint_value_target(tgt)
            self.arm_D.go()

    def _go_home_dual(self):
        """Home both arms with one trajectory on the dual group."""
        if self.arm_D is not None:
            tgt = {}
            tgt.update(self.arm_R.get_named_target_values("Rhome"))
            tgt.update(self.arm_L.get_named_target_values("Lhome"))
            self.arm_D.set_joint_value_target(tgt)
            self.arm_D.go()
        else:
            self._go_home()  # fall back