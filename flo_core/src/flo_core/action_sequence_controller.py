# action_sequence_controller.py
import rospy
from enum import IntEnum

class Action(IntEnum):
    HOME               = 0
    # Dynamic (repeated) actions
    D_WAVE             = 10
    D_RAISE            = 11
    D_SWING_LATERAL    = 12
    D_SWING_FORWARD    = 13
    # Static (single) actions
    S_RAISE            = 20
    S_REACH_SIDE       = 21
    S_TOUCH_HEAD       = 22
    S_TOUCH_MOUTH      = 23
    # … extend with more actions as needed …

class ActionSequenceController:
    def __init__(self, arm_R, arm_L):
        """
        arm_R: MoveGroupCommander for right arm
        arm_L: MoveGroupCommander for left arm
        """
        self.arm_R = arm_R
        self.arm_L = arm_L

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
        # placeholder: lateral swing implementation
        for _ in range(3):
            arm.set_named_target(f"{arm.get_name()}_swing_lat_left")
            arm.go()
            arm.set_named_target(f"{arm.get_name()}_swing_lat_right")
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
