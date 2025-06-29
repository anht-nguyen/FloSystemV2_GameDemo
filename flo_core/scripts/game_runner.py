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
from actionlib import SimpleActionClient

from polly_tts_streaming import PollyTTSStream


RULES_TEXT = (
    "Rules:\n"
    "  1) Robot will announce two arm actions.\n"
    "  2) If it says ‘Simon says’, you do them. Otherwise, you stay still.\n"
    "Click Continue when you’re ready, or Restart to hear these again."
)


WELCOME_SPEECH = (
    "Welcome to Simon Says game with Flo robot!\n")

RULES_SPEECH = """
    in simon says, I will tell you something to do and show you how to do it, mirrored.
    If I say simon says, you should do it with me.
    If I do not say simon says, you should not do the action.
    Watch out, I may try to trick you.
    After every movement return to a ready position.
    Are you ready to play?
    """

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

class Introduction:
    """
    Helper for the intro sequence: do a dual-arm wave, then publish the rules,
    and wait for the GUI to send ‘continue’ or ‘restart’.
    """
    def __init__(self, client: SimpleActionClient, prompt_pub, controller):
        self.client = client
        self.prompt_pub = prompt_pub
        self.controller = controller

    def run(self):
        # 1) Dual-arm wave
        wave_goal = SimonCmdGoal(gesture_name="D_WAVE_left|D_WAVE_right", simon_says=True)
        rospy.loginfo("[INTRO] sending dual-arm wave")
        self.client.send_goal(wave_goal)
        self.client.wait_for_result()

        # 2) Publish rules text
        rospy.loginfo("[INTRO] publishing rules")
        self.prompt_pub.publish(RULES_TEXT)

        # 3) Now wait for GUI to send 'continue' or 'restart'
        rospy.loginfo("[INTRO] awaiting confirmation")
        # control_msgs on /simon_game/control will set intro_done and last_cmd
        # we just spin until control_cb flips self._confirmed
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.controller._last_intro_cmd in ("continue", "restart"):
                return self.controller._last_intro_cmd
            rate.sleep()


class Announce(smach.State):
    def __init__(self, prompt_pub: rospy.Publisher, turn_pub: rospy.Publisher, tts_client):
        super().__init__(
            outcomes=["succeeded", "aborted"],
            input_keys=["left_action", "right_action", "simon_says", "turn_idx"],
            output_keys=["left_action", "right_action", "simon_says", "turn_idx"],
        )
        self.prompt_pub = prompt_pub
        self.turn_pub = turn_pub
        self.tts_client = tts_client

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
        # Speak the prompt using Amazon Polly
        self.tts_client.speak(prompt)
        # Publish the turn index
        self.turn_pub.publish(ud.turn_idx)
        return "succeeded"

# class WaitForPose(smach.State):
#     def __init__(self):
#         super().__init__(
#             outcomes=["matched", "timeout", "preempted"],
#             input_keys=["turn_timeout"],
#             output_keys=["pose_matched"],
#         )
#         self._latest_match = False
#         rospy.Subscriber("/arm_pose_score", PoseScore, self._cb)

#     def _cb(self, msg: PoseScore):
#         if msg.matched:
#             self._latest_match = True

#     def execute(self, ud):
#         self._latest_match = False
#         start = rospy.Time.now()
#         rate = rospy.Rate(30)
#         while not rospy.is_shutdown():
#             if self._latest_match:
#                 ud.pose_matched = True
#                 return "matched"
#             if (rospy.Time.now() - start).to_sec() > ud.turn_timeout:
#                 ud.pose_matched = False
#                 return "timeout"
#             if self.preempt_requested():
#                 self.service_preempt()
#                 ud.pose_matched = False
#                 return "preempted"
#             rate.sleep()


class WaitForPose(smach.State):
    def __init__(self):
        super().__init__(
            outcomes=["matched", "timeout", "preempted"],
            input_keys=["turn_timeout", "success_threshold"],
            output_keys=["pose_matched"],
        )

        self._total_poses = 0
        self._matched_poses = 0
        self._latest_match = False
        self._sub = rospy.Subscriber("/arm_hand_tracker/pose_score", PoseScore, self._cb)

    def _cb(self, msg: PoseScore):
        self._total_poses +=1
        if msg.matched:
            self._matched_poses += 1

    def execute(self, ud):
        self._total_poses = 0
        self._matched_poses = 0
        start = rospy.Time.now()
        rate = rospy.Rate(30)


        while not rospy.is_shutdown():
            # exit if preempt requested
            if self.preempt_requested():
                self.service_preempt()
                ud.pose_matched = False
                return "preempted"


            # Check if the time exceed the duraion (defaultly 5 sec)
            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed >= ud.turn_timeout:
                ratio = float(self._matched_poses) / max(self._total_poses, 1)
                matched = (ratio > ud.success_threshold)
                ud.pose_matched = matched
                return "matched" if matched else "timeout"
            rate.sleep()

class EvaluateState(smach.State):
    def __init__(self, score_pub: rospy.Publisher):
        super().__init__(
            outcomes=["good", "bad"],
            input_keys=["pose_matched", "score"],
            output_keys=["score", "eval_outcome"],
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

class NextTurnFromSequence(smach.State):
    """
    Advances turn_idx and loads the pre-generated gesture from 'sequence'.
    """
    def __init__(self, sequence: list[tuple[Action,Action,bool]]):
        super().__init__(
            outcomes=["continue", "finished"],
            input_keys=["turn_idx", "total_rounds"],
            output_keys=["turn_idx", "left_action", "right_action", "simon_says"],
        )
        self._sequence = sequence

    def execute(self, ud):
        ud.turn_idx += 1
        if ud.turn_idx > ud.total_rounds:
            return "finished"
        l, r, s = self._sequence[ud.turn_idx - 1]
        ud.left_action  = l
        ud.right_action = r
        ud.simon_says   = s
        return "continue"

class PauseWaitState(smach.State):
    """
    Waits for the game to be paused, then waits for a resume command.
    """
    def __init__(self, controller: GameController):
        super().__init__(outcomes=["resumed"])
        self.controller = controller

    def execute(self, ud):
        rospy.loginfo("[PAUSE] Game paused. Waiting for resume...")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.controller.resume_pending:
                self.controller.resume_pending = False
                self.controller.pause_pending = False
                rospy.loginfo("[PAUSE] Resuming game.")
                return "resumed"
            rate.sleep()


class PauseAfterEvaluateState(smach.State):
    def __init__(self, controller: GameController):
        super().__init__(outcomes=["good", "bad"],
                         input_keys=["eval_outcome"])
        self.controller = controller

    def execute(self, ud):
        rospy.loginfo("[PAUSE] Game paused after EVALUATE. Waiting to resume...")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.controller.resume_pending:
                self.controller.resume_pending = False
                self.controller.pause_pending = False
                rospy.loginfo(f"[PAUSE] Resuming. Routing to outcome: {ud.eval_outcome}")
                return ud.eval_outcome
            rate.sleep()




# ────────────────────────────────────────────────────────────────────────────
# Build SMACH container
# ────────────────────────────────────────────────────────────────────────────

def build_sm(sequence: list[tuple[Action,Action,bool]], params, score_pub, prompt_pub, controller: GameController):
    sm = smach.StateMachine(outcomes=["GAME_OVER"])
    with sm:
        sm.userdata.turn_idx = 0
        sm.userdata.score = 0
        sm.userdata.turn_timeout = params["turn_timeout"]
        sm.userdata.total_rounds = params["total_rounds"]
        sm.userdata.simon_ratio = params["simon_ratio"]
        sm.userdata.face_duration = params["face_duration"]
        sm.userdata.success_threshold = params["success_threshold"]
        sm.userdata.pose_matched = False
        # seed the first turn from our sequence
        first_l, first_r, first_s = sequence[0]
        sm.userdata.left_action   = first_l
        sm.userdata.right_action  = first_r
        sm.userdata.simon_says    = first_s
        
        turn_pub = rospy.Publisher('/simon_game/turn_id', Int32, queue_size=1)

        # Create PauseWaitState instance
        pause_state = PauseWaitState(controller)

        class WaitForPoseWithPause(WaitForPose):
            def execute(self, ud):
                outcome = super().execute(ud)
                if outcome == "matched" and controller.pause_pending:
                    return "PAUSE_AFTER_WAIT"
                return outcome
            
        # announce_and_cmd = smach.Concurrence(
        #     outcomes=["succeeded","aborted","preempted"],
        #     default_outcome="aborted",
        #     outcome_map={"succeeded": {"TALK":"succeeded","CMD":"succeeded"},
        #                  "aborted":   {"CMD":"aborted","TALK":"aborted"},
        #                  "preempted": {"CMD":"preempted"}},
        #     input_keys=["left_action","right_action","simon_says","turn_idx"],
        # )
        # with announce_and_cmd:
        #     smach.Concurrence.add("TALK", Announce(prompt_pub, turn_pub),)
        #     smach.Concurrence.add("CMD",
        #         smach_ros.SimpleActionState(
        #             "/simon_cmd", SimonCmdAction,
        #             goal_cb=_goal_cb,
        #             input_keys=["left_action","right_action","simon_says"],
        #             exec_timeout=rospy.Duration(15.0)
        #         )
        #     )

        # smach.StateMachine.add("WAIT_MOVE", WaitForPoseWithPause(),
        #                        transitions={"matched":"EVALUATE","timeout":"FAIL","preempted":"FAIL"})

        announce_cmd_and_detect = smach.Concurrence(
            outcomes=["matched", "timeout", "preempted", "aborted"],
            default_outcome="timeout",
            outcome_map={
                "matched": {"POSE":"matched"},
                "timeout": {"POSE":"timeout"},
                "preempted": {"POSE":"preempted"},
                "aborted": {"TALK":"aborted", "CMD":"aborted"}
            },
            input_keys=["left_action", "right_action", "simon_says", "turn_idx", "turn_timeout", "success_threshold","pose_matched"],
            output_keys=["pose_matched"],
            # child_termination_cb=lambda so: True  # terminate all when any completes
            child_termination_cb=lambda outcome_map:outcome_map.get("POSE") in ("matched","timeout")
        )

        with announce_cmd_and_detect:
            smach.Concurrence.add("TALK", Announce(prompt_pub, turn_pub, controller.tts))
            smach.Concurrence.add("CMD", smach_ros.SimpleActionState(
                "/simon_cmd", SimonCmdAction,
                goal_cb=_goal_cb,
                input_keys=["left_action", "right_action", "simon_says"],
                exec_timeout=rospy.Duration(15.0)
            ))
            smach.Concurrence.add("POSE", WaitForPoseWithPause())

        smach.StateMachine.add("ANNOUNCE", announce_cmd_and_detect,
            transitions={
                "matched": "EVALUATE",
                "timeout": "FAIL",
                "preempted": "FAIL",
                "aborted": "FAIL"
            })

        # Pause after WAIT_MOVE
        smach.StateMachine.add("PAUSE_AFTER_WAIT", pause_state,
                            transitions={"resumed": "EVALUATE"})
        
        class EvaluateStateWithPause(EvaluateState):
            def execute(self, ud):
                result = super().execute(ud)
                ud.eval_outcome = result  # ← record it
                if result == "good" and controller.pause_pending:
                    return "PAUSE_AFTER_EVAL"
                return result

        smach.StateMachine.add("EVALUATE", EvaluateStateWithPause(score_pub),
                               transitions={"good":"REWARD","bad":"FAIL"})

        # Pause after EVALUATE
        smach.StateMachine.add("PAUSE_AFTER_EVAL", PauseAfterEvaluateState(controller),
                                transitions={"good": "REWARD", "bad": "FAIL"})

        smach.StateMachine.add("REWARD", PublishEmotionState(Emotion.HAPPY), transitions={"done":"NEXT_TURN"})
        smach.StateMachine.add("FAIL", PublishEmotionState(Emotion.SAD), transitions={"done":"NEXT_TURN"})

        class NextTurnWithPause(NextTurnFromSequence):
            def __init__(self, sequence):
                super().__init__(sequence)
                self._outcomes = ["continue", "finished", "PAUSE_AFTER_NEXT"]

            def execute(self, ud):
                result = super().execute(ud)
                if result == "continue" and controller.pause_pending:
                    return "PAUSE_AFTER_NEXT"
                return result

        # NextTurnWithPause state
        smach.StateMachine.add("NEXT_TURN", NextTurnWithPause(sequence),
            transitions={"continue": "ANNOUNCE", 
                        "finished": "GAME_OVER",
                        "PAUSE_AFTER_NEXT": "PAUSE_AFTER_NEXT"})
        # Pause after NEXT_TURN
        smach.StateMachine.add("PAUSE_AFTER_NEXT", pause_state,
                            transitions={"resumed": "ANNOUNCE"})
    

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
            "turn_timeout": int(rospy.get_param("~turn_timeout")),
            "face_duration": rospy.get_param("~face_duration"),
            "simon_ratio": rospy.get_param("~simon_ratio"),
            "total_rounds": rospy.get_param("~total_rounds"),
            "success_threshold": rospy.get_param("~threshold")
        }

        # ── status publisher so GUI can enable “Start” when we’re ready ──
        self.status_pub = rospy.Publisher(
            "/simon_game/status", String, queue_size=1, latch=True
        )

        # ── PRE-GENERATE FULL SEQUENCE ─────────────────────────────────
        self.sequence = []
        for i in range(self.params["total_rounds"]):
            # pick once per turn
            l, r = _pick_actions(self.action_pool)
            s = (random.random() < self.params["simon_ratio"])
            self.sequence.append((l, r, s))

        # Print out entire sequence for debugging
        rospy.loginfo("[game_runner] Pre-generated gesture sequence:")
        for idx, (l, r, s) in enumerate(self.sequence, start=1):
            rospy.loginfo(f"  Turn {idx:2d}: {l.name}_left | {r.name}_right  SimonSays={s}")
        # ────────────────────────────────────────────────────────────────

        # Publishers
        self.score_pub = rospy.Publisher('/simon_game/score', Int32, queue_size=10)
        self.prompt_pub = rospy.Publisher('/simon_game/prompt', String, queue_size=1)

        # Initialize Amazon Polly TTS client
        self.tts = PollyTTSStream(voice_id="Salli", region_name="us-east-1")

        # Build state machine, passing in our fixed sequence
        self.sm = build_sm(self.sequence, self.params, self.score_pub, self.prompt_pub, self)
        # Introspection for viz
        self.sis = smach_ros.IntrospectionServer("game_sm", self.sm, "/GAME_SM")
        # Control subscriber
        self.control_sub = rospy.Subscriber('/simon_game/control', String, self.control_cb)
        self.running = False
        # state for intro handshake
        self.intro_done = False
        self._last_intro_cmd = None
        # Flags for pause/resume 
        self.pause_pending = False
        self.resume_pending = False
        # intro-thread state flags
        self.intro_in_progress = False
        # action-client for intro waving
        self.cmd_client = SimpleActionClient("/simon_cmd", SimonCmdAction)
        self.cmd_client.wait_for_server()
        # Start the introspection server
        self.game_thread = None            # keep a handle so we can join()

        self.cmd_client = SimpleActionClient("/simon_cmd", SimonCmdAction)
        self.cmd_client.wait_for_server()


    # ───────────────────────────────── INTRO HANDLER ─────────────────────────
    def _run_intro(self):
        """Runs in a background thread: dual-arm wave → rules → wait for GUI."""
        try:
            # ── 1)  WELCOME ────────────────────────────────────────────────
            rospy.loginfo("[INTRO] Speaking welcome speech")
            # Let the GUI show the same text via its /prompt callback
            # self.prompt_pub.publish(WELCOME_SPEECH.strip())

            # --- Speak & wave concurrently --------------------------------
            tts_thread = threading.Thread(
                target=self.tts.speak, args=(WELCOME_SPEECH,)
            )
            tts_thread.start()              # non-blocking TTS stream

            # Wave  (send_goal returns immediately; the arm starts moving)
            goal = SimonCmdGoal(
                gesture_name="D_WAVE_left|D_WAVE_right", simon_says=True
            )
            rospy.loginfo("[INTRO] Dual-arm wave")
            self.cmd_client.send_goal(goal)

            # Wait for BOTH the arm motion and the speech to finish
            self.cmd_client.wait_for_result()
            tts_thread.join()

            # 2) Rules
            rospy.loginfo("[INTRO] Publishing rules text")
            self.prompt_pub.publish(RULES_TEXT)
            rospy.loginfo("[INTRO] Speaking rules speech")
            self.tts.speak(RULES_SPEECH)

            # 3) Wait for GUI to press Continue / Restart
            r = rospy.Rate(10)
            while not rospy.is_shutdown():
                if self._last_intro_cmd == "continue":
                    rospy.loginfo("[INTRO] Continue received → start game")
                    self.intro_done = True
                    self.intro_in_progress = False
                    self._last_intro_cmd = None
                    # launch state-machine
                    self.running = True
                    self.sis.start()
                    threading.Thread(target=self.run_game).start()
                    return
                if self._last_intro_cmd == "restart":
                    rospy.loginfo("[INTRO] Restart received → reset intro")
                    self.intro_in_progress = False
                    self._last_intro_cmd = None
                    # ── tell GUI everything is reset ───────────────────────
                    self.score_pub.publish(0)                          # scoreboard → 0
                    rospy.Publisher('/simon_game/turn_id', Int32,
                                    queue_size=1, latch=True).publish(0)
                    self.status_pub.publish("Waiting for Start command..")
                    return
                r.sleep()
        finally:
            # safety: clear flag on any unexpected exit
            self.intro_in_progress = False

    # ───────────────────────────────── CONTROL CALLBACK ─────────────────────
    def control_cb(self, msg: String):
        cmd = msg.data
        rospy.loginfo(f"Received control: {cmd}")

        # ----- pre-game: introduction handshake -----
        if not self.intro_done:
            if cmd == "start" and not self.intro_in_progress:
                # spawn intro thread and return immediately
                self.intro_in_progress = True
                threading.Thread(target=self._run_intro, daemon=True).start()
            elif cmd in ("continue", "restart") and self.intro_in_progress:
                # just record the GUI choice; intro thread will react
                self._last_intro_cmd = cmd
            return  # ignore everything else until intro is settled

        # ----- post-intro: normal game controls -----
        if cmd == 'start' and not self.running:
            self.running = True
            self.sis.start()
            self.game_thread = threading.Thread(target=self.run_game, daemon=True)
            self.game_thread.start()
        # Pause/Resume controls
        elif cmd == 'pause' and self.running:
            self.pause_pending = True
            self.sm.request_preempt()
        elif cmd == 'continue':
            self.resume_pending = True
            self.pause_pending = False
        # Force finish
        elif cmd == 'stop' and self.running:
            self.sm.userdata.turn_idx = self.params['total_rounds'] + 1
        elif cmd == 'restart':
            rospy.loginfo("[game_runner] Restart requested – terminating current game")
            # ── 1)  PREEMPT & cancel everything ──────────────────────────
            if self.running:
                # Force the state-machine to finish on the next step.
                self.sm.userdata.turn_idx = self.params['total_rounds'] + 1
                self.sm.request_preempt()

                # Make sure no arm-gesture goal is still running.
                try:
                    self.cmd_client.cancel_all_goals()
                except Exception:
                    pass

                # Give the thread a moment to exit (longer than a single turn timeout).
                if self.game_thread:
                    self.game_thread.join(timeout=5.0)
                self.running = False

            # -- 1b) Stop & replace the old introspection server ----------------
            try:
                self.sis.stop()
            except Exception:
                pass

            # 2) Clear intro handshake so rules are shown again
            self.intro_done = False
            self.intro_in_progress = False
            self._last_intro_cmd = None

            # 3) Publish reset values so the GUI shows   Turn: 0 / Score: 0
            self.score_pub.publish(0)
            rospy.Publisher('/simon_game/turn_id', Int32, queue_size=1, latch=True).publish(0)

            # 4) Build a brand-new gesture sequence and SMACH graph
            self.sequence = []
            for _ in range(self.params["total_rounds"]):
                l, r = _pick_actions(self.action_pool)
                s = (random.random() < self.params["simon_ratio"])
                self.sequence.append((l, r, s))

            self.sm = build_sm(
                self.sequence, self.params, self.score_pub, self.prompt_pub, self
            )

            # 5) Fresh introspection server bound to the new SM
            self.sis = smach_ros.IntrospectionServer("game_sm", self.sm, "/GAME_SM")
            rospy.loginfo("[game_runner] Ready for new Start")

            # tell GUI it’s safe to re-enable the Start button
            self.status_pub.publish("Waiting for Start command..")

            return
        elif cmd == 'quit':
            rospy.signal_shutdown('Quit via GUI')

    # ───────────────────────────────── GAME LOOP ────────────────────────────
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
    # Console + GUI become ready together
    rospy.loginfo('[game_runner] Waiting for Start command...')
    controller.status_pub.publish("Waiting for Start command..")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
