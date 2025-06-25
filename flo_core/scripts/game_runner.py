#!/usr/bin/env python3
"""
game_runner.py – high-level game orchestrator for Responsive Simon Says
ROS Noetic | Python 3.x

* Publishes  GameAction  – what the robot should do next
* Publishes  GameState   – IDLE, RUNNING, SUCCESS, FAILURE, FINISHED
* Subscribes GameCommand – external control (start / stop / pause …)
* Subscribes PoseScore   – vision feedback for each trial
* Uses      GetPoseID    – resolve a single pose by numeric ID
* Uses      GetPoseSeqID – resolve a pose *sequence* (if StepDef.type=="sequence")

Author: 2025, FLO Robot project
"""

import yaml
import rospy
from std_msgs.msg import Header

# ---- custom interfaces ------------------------------------------------------
from flo_core_defs.msg import (                     # adjust package name if needed
    GameAction, GameCommand, GameState,
    PoseScore, StepDef, GameDef                      # PoseScore.msg lives in flo_core_defs too
)
from flo_core_defs.srv import (
    GetPoseID,       GetPoseIDRequest,
    GetPoseSeqID,    GetPoseSeqIDRequest,
)

STATE_IDLE      = "IDLE"
STATE_RUNNING   = "RUNNING"
STATE_SUCCESS   = "SUCCESS"
STATE_FAILURE   = "FAILURE"
STATE_FINISHED  = "FINISHED"

class GameRunner:
    """High-level state machine for Simon Says."""
    def __init__(self):
        # --- parameters ---
        yaml_file         = rospy.get_param("~game_def_file", "config/simon_says.yaml")
        self.success_tol  = rospy.get_param("~success_threshold", 0.7)   # PoseScore.score ∈ [0,1]

        # --- load game definition ---
        self.game_def = self._load_game_def(yaml_file)
        self.current_step_idx = 0
        self.current_rep      = 0
        self.in_progress      = False

        # --- comms ---
        self.pub_action = rospy.Publisher("game_action", GameAction, queue_size=10, latch=True)
        self.pub_state  = rospy.Publisher("game_state",  GameState,  queue_size=10, latch=True)

        rospy.Subscriber("game_command", GameCommand, self._command_cb, queue_size=10)
        rospy.Subscriber("pose_score",   PoseScore,   self._pose_score_cb, queue_size=10)

        # --- service proxies ---
        rospy.wait_for_service("get_pose_by_id")
        rospy.wait_for_service("get_pose_seq_by_id")
        self.srv_pose     = rospy.ServiceProxy("get_pose_by_id",     GetPoseID)
        self.srv_pose_seq = rospy.ServiceProxy("get_pose_seq_by_id", GetPoseSeqID)

        # publish initial state
        self._publish_state(STATE_IDLE)
        rospy.loginfo("✅  game_runner spun up and waiting for GameCommand…")

    # --------------------------------------------------------------------- #
    #  Callbacks                                                            #
    # --------------------------------------------------------------------- #
    def _command_cb(self, msg: GameCommand):
        cmd = msg.command.lower().strip()
        rospy.loginfo(f"[game_runner] Received command: {cmd}")
        if cmd == "start" and not self.in_progress:
            self.current_step_idx = 0
            self.current_rep      = 0
            self.in_progress      = True
            self._publish_state(STATE_RUNNING)
            self._dispatch_current_step()

        elif cmd == "stop":
            self.in_progress = False
            self._publish_state(STATE_IDLE)

        elif cmd == "pause":
            self.in_progress = False
            self._publish_state(STATE_IDLE)

        # add more verbs (“resume”, “skip”) as needed

    def _pose_score_cb(self, msg: PoseScore):
        if not self.in_progress:
            return

        if msg.score >= self.success_tol and msg.correct:
            self._publish_state(STATE_SUCCESS)
            rospy.sleep(0.5)                                   # brief joy/sad face window
            self._next_step()
        else:
            self._publish_state(STATE_FAILURE)
            rospy.sleep(1.0)                                   # longer feedback on failure
            self._next_step()                                  # still advance (or repeat—your choice)

    # --------------------------------------------------------------------- #
    #  Helpers                                                              #
    # --------------------------------------------------------------------- #
    def _next_step(self):
        self.current_step_idx += 1

        if self.current_step_idx >= len(self.game_def.steps):
            self.current_rep += 1
            self.current_step_idx = 0

        if self.current_rep >= self.game_def.reps:
            self.in_progress = False
            self._publish_state(STATE_FINISHED)
            rospy.loginfo("🎉  Simon Says session complete!")
            return

        self._publish_state(STATE_RUNNING)
        self._dispatch_current_step()

    def _dispatch_current_step(self):
        step: StepDef = self.game_def.steps[self.current_step_idx]
        action        = GameAction()
        action.header = Header(stamp=rospy.Time.now())
        action.speech   = step.text
        action.step_id  = step.id

        # get target(s) from humanoid pose DB
        try:
            if step.type == "pose":
                resp = self.srv_pose(GetPoseIDRequest(id=step.id))
                action.targets = resp.pose.targets

            elif step.type == "sequence":
                resp = self.srv_pose_seq(GetPoseSeqIDRequest(id=step.id))
                # pick the first pose for scoring; send full sequence to robot
                action.targets = resp.sequence.poses

            else:
                rospy.logwarn(f"Unknown step.type='{step.type}', skipping.")
                self._next_step()
                return

            self.pub_action.publish(action)
            rospy.loginfo(f"[game_runner] Dispatched step {step.id} ({step.text})")

        except rospy.ServiceException as e:
            rospy.logerr(f"Pose lookup failed: {e}")
            self._publish_state(STATE_FAILURE)
            self.in_progress = False

    def _publish_state(self, s: str):
        state_msg = GameState()
        state_msg.state = s
        self.pub_state.publish(state_msg)

    # --------------------------------------------------------------------- #
    #  Util                                                                 #
    # --------------------------------------------------------------------- #
    @staticmethod
    def _load_game_def(path: str) -> GameDef:
        """Read YAML → GameDef message."""
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        gd = GameDef()
        gd.game_type = data.get("game_type", "SIMON_SAYS")
        gd.reps      = data.get("reps", 1)
        gd.max_steps = data.get("max_steps", len(data["steps"]))

        for step_dict in data["steps"]:
            sd = StepDef()
            sd.type = step_dict["type"]
            sd.text = step_dict["text"]
            sd.id   = int(step_dict["id"])
            sd.time = float(step_dict.get("time", 0))
            gd.steps.append(sd)

        return gd

# ------------------------------------------------------------------------- #
#  Main                                                                     #
# ------------------------------------------------------------------------- #
def main():
    rospy.init_node("game_runner")
    GameRunner()
    rospy.spin()

if __name__ == "__main__":
    main()
