#!/usr/bin/env python3
"""
FLO‑Core ─ game_runner.py (v6)
==============================
Integrated with GUI: listens to control commands, publishes live score and prompts.
"""
from __future__ import annotations
import random
import threading
import rospy
import smach
import smach_ros

from std_msgs.msg import String, Int32
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

def _goal_cb(ud, _):
    goal = SimonCmdGoal()
    goal.gesture_name = f"{ud.left_action.name}_left|{ud.right_action.name}_right"
    goal.simon_says = ud.simon_says
    return goal

class Announce(smach.State):
    def __init__(self, prompt_pub: rospy.Publisher, turn_pub: rospy.Publisher):
        super().__init__(
            outcomes=["succeeded", "aborted"],
            input_keys=["left_action", "right_action", "simon_says", "turn_idx"],
            output_keys=["left_action", "right_action", "simon_says", "turn_idx"],
        )
        self.prompt_pub = prompt_pub
        self.turn_pub = turn_pub

    def execute(self, ud):
        left = ud.left_action.name if ud.left_action else ""
        right = ud.right_action.name if ud.right_action else ""
        if not left and not right:
            rospy.logwarn("Announce: both actions empty")
            return "aborted"
        # Build and publish prompt
        prompt = build_prompt(left, right, ud.simon_says)
        rospy.loginfo(f"Prompt: {prompt}")
        self.prompt_pub.publish(prompt)
        self.turn_pub.publish(ud.turn_idx)
        return "succeeded"

class WaitForPose(smach.State):
    def __init__(self):
        super().__init__(
            outcomes=["matched", "timeout", "preempted"],
            input_keys=["turn_timeout"],
            output_keys=["pose_matched"],
        )
        self._latest_match = False
        rospy.Subscriber("/pose_score", PoseScore, self._cb)

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
    def __init__(self, score_pub: rospy.Publisher):
        super().__init__(
            outcomes=["good", "bad"],
            input_keys=["pose_matched", "score"],
            output_keys=["score"],
        )
        self.score_pub = score_pub

    def execute(self, ud):
        if ud.pose_matched:
            ud.score += 1
            self.score_pub.publish(ud.score)
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
    def __init__(self, action_pool: list[Action]):
        super().__init__(
            outcomes=["continue", "finished"],
            input_keys=["turn_idx", "total_rounds", "left_action", "right_action", "simon_ratio"],
            output_keys=["left_action", "right_action", "simon_says", "turn_idx", "turn_timeout"],
        )
        self._pool = action_pool

    def execute(self, ud):
        ud.turn_idx += 1
        if ud.turn_idx > ud.total_rounds:
            return "finished"
        ud.left_action, ud.right_action = _pick_actions(self._pool)
        ud.simon_says = random.random() < ud.simon_ratio
        # turn_timeout remains same from userdata
        return "continue"

# ────────────────────────────────────────────────────────────────────────────
# Build SMACH container
# ────────────────────────────────────────────────────────────────────────────

def build_sm(action_pool: list[Action], params, score_pub, prompt_pub):
    sm = smach.StateMachine(outcomes=["GAME_OVER"])
    with sm:
        sm.userdata.turn_idx = 0
        sm.userdata.score = 0
        sm.userdata.turn_timeout = params["turn_timeout"]
        sm.userdata.total_rounds = params["total_rounds"]
        sm.userdata.simon_ratio = params["simon_ratio"]
        sm.userdata.face_duration = params["face_duration"]
        sm.userdata.simon_says = True
        sm.userdata.pose_matched = False
        sm.userdata.left_action, sm.userdata.right_action = _pick_actions(action_pool)
        sm.userdata.simon_says = random.random() < sm.userdata.simon_ratio
        
        turn_pub = rospy.Publisher('/simon_game/turn_id', Int32, queue_size=1)

        announce_and_cmd = smach.Concurrence(
            outcomes=["succeeded","aborted","preempted"],
            default_outcome="aborted",
            outcome_map={"succeeded": {"TALK":"succeeded","CMD":"succeeded"},
                         "aborted":   {"CMD":"aborted","TALK":"aborted"},
                         "preempted": {"CMD":"preempted"}},
            input_keys=["left_action","right_action","simon_says","turn_idx"],
        )
        with announce_and_cmd:
            smach.Concurrence.add("TALK", Announce(prompt_pub, turn_pub),)
            smach.Concurrence.add("CMD",
                smach_ros.SimpleActionState(
                    "/simon_cmd", SimonCmdAction,
                    goal_cb=_goal_cb,
                    input_keys=["left_action","right_action","simon_says"],
                    exec_timeout=rospy.Duration(15.0)
                )
            )
        smach.StateMachine.add("ANNOUNCE", announce_and_cmd,
                               transitions={"succeeded":"WAIT_MOVE","aborted":"FAIL","preempted":"FAIL"})
        smach.StateMachine.add("WAIT_MOVE", WaitForPose(),
                               transitions={"matched":"EVALUATE","timeout":"FAIL","preempted":"FAIL"})
        smach.StateMachine.add("EVALUATE", EvaluateState(score_pub),
                               transitions={"good":"REWARD","bad":"FAIL"})
        smach.StateMachine.add("REWARD", PublishEmotionState(Emotion.HAPPY), transitions={"done":"NEXT_TURN"})
        smach.StateMachine.add("FAIL", PublishEmotionState(Emotion.SAD), transitions={"done":"NEXT_TURN"})
        smach.StateMachine.add("NEXT_TURN", NextTurnState(action_pool),
                               transitions={"continue":"ANNOUNCE","finished":"GAME_OVER"})
    return sm

# ────────────────────────────────────────────────────────────────────────────
# Control Handler
# ────────────────────────────────────────────────────────────────────────────

class GameController:
    def __init__(self):
        # Params & action pool
        default_actions = [a.name for a in Action]
        left = rospy.get_param("~left_actions", default_actions)
        right = rospy.get_param("~right_actions", default_actions)
        pool = {Action[n] for n in left+right if n in Action.__members__}
        self.action_pool = list(pool)

        self.params = {
            "turn_timeout": int(rospy.get_param("~turn_timeout", 4.0)),
            "face_duration": rospy.get_param("~face_duration", 1.2),
            "simon_ratio": rospy.get_param("~simon_ratio", 0.6),
            "total_rounds": rospy.get_param("~total_rounds", 15),
        }
        # Publishers
        self.score_pub = rospy.Publisher('/simon_game/score', Int32, queue_size=10)
        self.prompt_pub = rospy.Publisher('/simon_game/prompt', String, queue_size=1)
        # Build state machine
        self.sm = build_sm(self.action_pool, self.params, self.score_pub, self.prompt_pub)
        # Introspection for viz
        self.sis = smach_ros.IntrospectionServer("game_sm", self.sm, "/GAME_SM")
        # Control subscriber
        self.control_sub = rospy.Subscriber('/simon_game/control', String, self.control_cb)
        self.running = False

    def control_cb(self, msg: String):
        cmd = msg.data
        rospy.loginfo(f"Received control: {cmd}")
        if cmd == 'start' and not self.running:
            self.running = True
            self.sis.start()
            threading.Thread(target=self.run_game).start()
        elif cmd == 'pause' and self.running:
            self.sm.request_preempt()
        elif cmd == 'stop' and self.running:
            # Force finish
            self.sm.userdata.turn_idx = self.params['total_rounds'] + 1
        elif cmd == 'quit':
            rospy.signal_shutdown('Quit via GUI')

    def run_game(self):
        outcome = self.sm.execute()
        rospy.loginfo(f"Game finished: {outcome}")
        self.sis.stop()
        self.running = False

# ────────────────────────────────────────────────────────────────────────────
# Entry point
# ────────────────────────────────────────────────────────────────────────────

def main():
    rospy.init_node('game_runner')
    controller = GameController()
    rospy.loginfo('[game_runner] Waiting for Start command...')
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
