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

from std_msgs.msg import String, Int32, Bool
from flo_core_defs.msg import PoseScore, Emotion
from flo_core_defs.msg import SimonCmdAction, SimonCmdGoal
from flo_core_defs.msg import CalibStatus
from flo_core.action_sequence_controller import Action
from flo_core.prompt_utils import build_prompt
from actionlib import SimpleActionClient

from flo_core.polly_tts_streaming import PollyTTSStream


RULES_TEXT = (
    "Rules:\n"
    "  1) Robot will announce two arm actions.\n"
    "  2) If it says ‘Simon says’, you do them. Otherwise, you stay still.\n"
    "Use the GUI buttons to replay instructions, calibrate the camera, or start the game."
)


WELCOME_SPEECH = (
    "Welcome to Simon Says game with Flo robot!\n")

RULES_SPEECH = """
    In simon says, I will tell you something to do and show you how to do it in a mirrored style.
    If I say simon says, you should do it with me.
    If I do not say simon says, you should not do the action.
    Watch out, I may try to trick you.
    After every movement return to a ready position.
    Are you ready to play? 
    """

READY_STATUS = "Ready for setup or game start."

# ────────────────────────────────────────────────────────────────────────────
# Helper states
# ────────────────────────────────────────────────────────────────────────────

def _pick_actions(pool):
    a_left = random.choice(pool)
    a_right = random.choice(pool)
    # Enable different actions for left and right arms
    # while len(pool) > 1 and a_left == a_right:
    #     a_right = random.choice(pool)
    return a_left, a_right

def _goal_cb(ud, _):
    # -- Delay to ensure any prior goal cancel has been processed --
    # You can tune this via ROS param '~cmd_goal_delay' (in seconds)
    try:
        delay = float(rospy.get_param('~cmd_goal_delay', 0.5))
    except (KeyError, ValueError):
        delay = 0.5
    rospy.logdebug(f"Delaying {delay}s before sending simon_cmd goal")
    rospy.sleep(rospy.Duration(delay))
    goal = SimonCmdGoal()
    goal.gesture_name = f"{ud.left_action.name}_left|{ud.right_action.name}_right"
    goal.simon_says = ud.simon_says
    return goal

class CalibrationStage:
    """
    Drives the 'step back / forward' dialogue until flo_vision
    reports that the full upper body + arm-over-head are in view.
    """
    def __init__(self, tts, prompt_pub, cancel_event: threading.Event | None = None):
        self._tts = tts
        self._prompt_pub = prompt_pub
        self._cancel_event = cancel_event
        self._speech_lock = threading.Lock()
        self._speech_thread = None
        self._pending_speech = None
        self._status_sub = rospy.Subscriber(
            "/simon_game/calib_status", CalibStatus, self._status_cb
        )
        self._ready = False
        self._hint  = ""

        # Publisher that toggles calibration mode inside flo_vision
        self._cmd_pub = rospy.Publisher(
            "/simon_game/calib_cmd", Bool, queue_size=1, latch=True
        )

    def _speech_worker(self, text: str):
        current = text
        while current and not rospy.is_shutdown():
            self._tts.speak(current)
            with self._speech_lock:
                if self._cancel_event and self._cancel_event.is_set():
                    self._pending_speech = None
                    self._speech_thread = None
                    return
                if self._pending_speech and self._pending_speech != current:
                    current = self._pending_speech
                    self._pending_speech = None
                    continue
                self._pending_speech = None
                self._speech_thread = None
                return

    def _speak_async(self, text: str):
        with self._speech_lock:
            if self._speech_thread and self._speech_thread.is_alive():
                self._pending_speech = text
                return
            self._pending_speech = None
            self._speech_thread = threading.Thread(
                target=self._speech_worker, args=(text,), daemon=True
            )
            self._speech_thread.start()

    def _wait_for_speech(self):
        while not rospy.is_shutdown():
            with self._speech_lock:
                speech_thread = self._speech_thread
                pending_speech = self._pending_speech
            if speech_thread is None and pending_speech is None:
                return
            if self._cancel_event and self._cancel_event.is_set():
                return
            if speech_thread is not None:
                speech_thread.join(timeout=0.1)
            else:
                rospy.sleep(0.05)

    # ------------------------------------------------------------------
    def _status_cb(self, msg: CalibStatus):
        self._ready = msg.ready
        self._hint  = msg.hint.lower()

    # ------------------------------------------------------------------
    def run(self):
        # 1) Tell GUI + Vision we’re entering calibration
        self._cmd_pub.publish(True)
        self._prompt_pub.publish(
            "Please stand in front of me about 5 feet, or 1.5 meters, away. "
            "Then let's do a quick camera check."
        )
        self._speak_async(
            "Please stand in front of me about 5 feet, or 1.5 meters, away. "
            "Let's first make sure the camera can see you well. "
            "Please raise your arm fully overhead and hold it there."
        )

        rate = rospy.Rate(10)
        stage = 1           # 1 = checking pose, 2 = checking framing
        last_hint = None

        while not rospy.is_shutdown():
            if self._cancel_event and self._cancel_event.is_set():
                self._cmd_pub.publish(False)
                return False

            # Stage 1 → Stage 2 as soon as we get any hint *besides* "raise_arm"
            if stage == 1 and self._hint == "arm_up":
                # Move to framing stage
                stage = 2
                self._prompt_pub.publish(
                    "Great! Now step back or forward so I can see your full upper body."
                )
                self._speak_async(
                    "Great! Now step back or forward until I can see your whole upper body and raised arm."
                )
                last_hint = None

            if stage == 2 and self._ready and self._hint == "arm_up":
                # Success!
                self._speak_async("Perfect! I can see your whole upper body.")
                self._wait_for_speech()
                self._cmd_pub.publish(False)
                return True

            # Only speak when hint changes
            if self._hint and self._hint != last_hint:
                msg = {
                    "raise_arm":    "Please raise your arm fully overhead.",
                    "back":      "A little farther, please.",
                    "forward":   "Come a bit closer.",
                    "left":      "Move slightly to your left.",
                    "right":     "Move slightly to your right.",
                }.get(self._hint, "")
                if msg:
                    self._prompt_pub.publish(msg)
                    self._speak_async(msg)
                    last_hint = self._hint

            rate.sleep()

        self._cmd_pub.publish(False)
        return False


class StaticPoseStartStage:
    """
    Runs a short static-pose instruction, then verifies the player holds it
    before the actual game starts.
    """
    def __init__(
        self,
        tts,
        prompt_pub,
        success_threshold: float,
        hold_seconds: float,
        cancel_event: threading.Event | None = None,
    ):
        self._tts = tts
        self._prompt_pub = prompt_pub
        self._success_threshold = success_threshold
        self._hold_seconds = hold_seconds
        self._cancel_event = cancel_event
        self._latest_match = False
        self._match_started_at = None
        self._sub = rospy.Subscriber(
            "/arm_hand_tracker/pose_score", PoseScore, self._pose_cb
        )
        self._pose_cmd_pub = rospy.Publisher(
            "/arm_hand_tracker/pose_command", String, queue_size=1, latch=True
        )

    def _pose_cb(self, msg: PoseScore):
        self._latest_match = (
            msg.matched and msg.similarity >= self._success_threshold
        )

    def _run_instruction_step(self) -> bool:
        self._prompt_pub.publish(
            "Before we start, your static pose is both arms straight and relaxed by your sides."
        )
        self._tts.speak(
            "Before we start, your static pose is both arms straight and relaxed by your sides."
        )
        if self._cancel_event and self._cancel_event.is_set():
            return False
        rospy.sleep(0.5)
        return not (self._cancel_event and self._cancel_event.is_set())

    def _run_pose_check_step(self) -> bool:
        self._latest_match = False
        self._match_started_at = None
        self._pose_cmd_pub.publish(String(data="static"))
        self._prompt_pub.publish(
            "Show me your static pose now and hold it still for a moment."
        )
        self._tts.speak(
            "Show me your static pose now and hold it still for a moment."
        )

        rate = rospy.Rate(20)
        reminded_at = rospy.Time.now()

        while not rospy.is_shutdown():
            if self._cancel_event and self._cancel_event.is_set():
                return False

            now = rospy.Time.now()
            if self._latest_match:
                if self._match_started_at is None:
                    self._match_started_at = now
                if (now - self._match_started_at).to_sec() >= self._hold_seconds:
                    self._prompt_pub.publish("Static pose looks good. Starting the game.")
                    self._tts.speak("Static pose looks good. Starting the game.")
                    return True
            else:
                self._match_started_at = None
                if (now - reminded_at).to_sec() >= 4.0:
                    self._prompt_pub.publish(
                        "Please put both arms straight by your sides to make your static pose."
                    )
                    self._tts.speak(
                        "Please put both arms straight by your sides to make your static pose."
                    )
                    reminded_at = now

            rate.sleep()

        return False

    def run(self) -> bool:
        try:
            if not self._run_instruction_step():
                return False
            return self._run_pose_check_step()
        finally:
            self._sub.unregister()


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
        self.pose_cmd_pub = rospy.Publisher("/arm_hand_tracker/pose_command", String, queue_size=1, latch = True)

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
        
        # Publish the turn index
        self.turn_pub.publish(ud.turn_idx)

        if not ud.simon_says:
            # If it is not a Simon Says turn, we will let camera detect static
            msg = String(data="static")
        else:
            # If it is a Simon Says turn, we will let camera detect the left and right actions
            player_right = ud.left_action.name.lower() +"_right"
            player_left = ud.right_action.name.lower() + "_left"
            msg = String(data=f"{player_right},{player_left}")
            # msg = String(data=f"{player_left},{player_right}")
        rospy.loginfo(f"Publishing pose command: {msg.data}")
        self.pose_cmd_pub.publish(msg)
        # Speak the prompt using Amazon Polly
        self.tts_client.speak(prompt)

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
        # Publish emotion (happy/sad)
        self._pub.publish(Emotion(state=self._val))
        # Hold face for configured duration
        rospy.sleep(rospy.Duration(ud.face_duration))
        # --- Insert inter-turn delay to give robot a pause before next turn
        # You can adjust this delay via ROS param '~inter_turn_delay' (in seconds)
        delay = float(rospy.get_param('~inter_turn_delay'))
        rospy.loginfo(f"Inter-turn delay: {delay}s")
        rospy.sleep(rospy.Duration(delay))
        return "done"

class FeedbackState(smach.State):
    """
    Publishes an emotion **and** (optionally) speaks a randomly-chosen phrase.
    """
    def __init__(self,
                 emotion_val: int,
                 phrases: list[str],
                 tts_client):
        super().__init__(outcomes=["done"],
                         input_keys=["face_duration"])
        self._emotion_val = emotion_val
        self._phrases     = phrases
        self._tts         = tts_client
        self._emotion_pub = rospy.Publisher(
            "/emotion", Emotion, queue_size=1, latch=True)

    # ------------------------------------------------------------------
    def execute(self, ud):
        # 1) show the face
        self._emotion_pub.publish(Emotion(state=self._emotion_val))

        # 2) randomly decide whether / what to say
        phrase = random.choice(self._phrases)
        if phrase:                           # empty string  ➜  silence
            self._tts.speak(phrase)

        # 3) keep the face for N seconds
        rospy.sleep(rospy.Duration(ud.face_duration))

        # 4) optional inter-turn delay
        delay = float(rospy.get_param('~inter_turn_delay', 0.0))
        rospy.loginfo(f"Inter-turn delay: {delay}s")
        rospy.sleep(rospy.Duration(delay))
        return "done"

class FailFeedbackState(smach.State):
    """
    Shows the sad face **and** speaks a context-aware quip.

    • If the last turn had simon_says == False  ➜  the player was tricked
      → choose from fail_lines_tricked  (“Gotcha!”, …).

    • Otherwise it was a normal miss  ➜  choose from fail_lines_missed.
    """
    def __init__(self, tts_client):
        super().__init__(
            outcomes=["done"],
            input_keys=["face_duration", "simon_says"],
        )
        self._pub = rospy.Publisher("/emotion", Emotion, queue_size=1, latch=True)
        self._tts = tts_client

        # ----- Edit these lists anytime ----------------------------------
        self.fail_lines_tricked = [
            "Gotcha!", "Simon didn’t say!", "Fooled you!", ""
        ]
        self.fail_lines_missed  = [
            "Not quite.", "Try the next one.", "Almost!", ""
        ]

    # ------------------------------------------------------------------
    def execute(self, ud):
        # 1) sad face
        self._pub.publish(Emotion(state=Emotion.SAD))

        # 2) pick a phrase
        pool   = self.fail_lines_tricked if not ud.simon_says else self.fail_lines_missed
        phrase = random.choice(pool)
        if phrase:                    # empty string  ➜  silent variant
            self._tts.speak(phrase)

        # 3) hold face, then normal inter-turn delay
        rospy.sleep(rospy.Duration(ud.face_duration))
        delay = float(rospy.get_param("~inter_turn_delay", 0.0))
        rospy.sleep(rospy.Duration(delay))
        return "done"


class ReturnHomeState(smach.State):
    """
    Sends a 'HOME' pose command after each turn so the robot
    always goes back to its starting position.
    """
    def __init__(self, action_client: SimpleActionClient):
        super().__init__(outcomes=["done"])
        self._client = action_client

    def execute(self, ud):
        # Build and send the HOME pose goal
        home_goal = SimonCmdGoal(
            gesture_name="HOME_left|HOME_right",   # name of your home pose in the action server
            simon_says=True        # doesn't matter for HOME, but must be set
        )
        rospy.loginfo("[RETURN_HOME] sending robot to HOME pose")
        self._client.send_goal(home_goal)
        self._client.wait_for_result()
        return "done"


class NextTurnFromSequence(smach.State):
    """
    Advances turn_idx and loads the pre-generated gesture from 'sequence'.
    """
    def __init__(self, sequence: list[tuple[Action,Action,bool]]):
        super().__init__(
            outcomes=["continue", "finished"],
            input_keys=["turn_idx", "total_rounds", 'static_threshold'],
            output_keys=["turn_idx", "left_action", "right_action", "simon_says", 'success_threshold'],
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
        if not s:
            ud.success_threshold = ud.static_threshold
        else:
            ud.success_threshold = rospy.get_param("~threshold")
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
        sm.userdata.turn_idx = 1
        sm.userdata.score = 0
        sm.userdata.turn_timeout = params["turn_timeout"]
        sm.userdata.total_rounds = params["total_rounds"]
        sm.userdata.simon_ratio = params["simon_ratio"]
        sm.userdata.face_duration = params["face_duration"]
        sm.userdata.success_threshold = params["success_threshold"]
        sm.userdata.static_threshold = params["static_threshold"]
        sm.userdata.pose_matched = False
        # seed the first turn from our sequence
        first_l, first_r, first_s = sequence[0]
        sm.userdata.left_action   = first_l
        sm.userdata.right_action  = first_r
        sm.userdata.simon_says    = first_s

        if not first_s:
             sm.userdata.success_threshold = sm.userdata.static_threshold
        
        turn_pub = rospy.Publisher('/simon_game/turn_id', Int32, queue_size=1)


        # ────────────────────────────────────────────────────────────────────
        #  New helper: builds a fresh CMD action-state every turn
        # ────────────────────────────────────────────────────────────────────
        class DynamicAnnounceCmdDetect(smach.State):
            """
            Re-creates the ANNOUNCE/TALK + CMD + POSE concurrence on *every*
            execute() so the SimpleActionState is always a new instance
            (no stale pre-empt flag can survive across turns).
            """
            def __init__(self, prompt_pub, turn_pub, tts_client):
                super().__init__(
                    outcomes=["matched", "timeout", "preempted", "aborted"],
                    input_keys=[
                        "left_action", "right_action", "simon_says",
                        "turn_idx", "turn_timeout", "success_threshold",
                        "pose_matched"
                    ],
                    output_keys=["pose_matched"],
                )
                self.prompt_pub = prompt_pub
                self.turn_pub   = turn_pub
                self.tts_client = tts_client

            # ------------------------------------------------------------------
            def execute(self, ud):
                # Build a brand-new Concurrence (and thus a brand-new CMD state)
                cc = smach.Concurrence(
                    outcomes=["matched", "timeout", "preempted", "aborted"],
                    default_outcome="timeout",
                    outcome_map={
                        "matched":   {"POSE": "matched"},
                        "timeout":   {"POSE": "timeout"},
                        "preempted": {"POSE": "preempted"},
                        "aborted":   {"TALK": "aborted", "CMD": "aborted"},
                    },
                    input_keys=[
                        "left_action", "right_action", "simon_says",
                        "turn_idx", "turn_timeout", "success_threshold",
                        "pose_matched"
                    ],
                    output_keys=["pose_matched"],
                    child_termination_cb=lambda om: om.get("POSE") in ("matched", "timeout"),
                )

                with cc:
                    smach.Concurrence.add(
                        "TALK",
                        Announce(self.prompt_pub, self.turn_pub, self.tts_client),
                    )
                    # ←--- *fresh* SimpleActionState every time ───────────────────
                    smach.Concurrence.add(
                        "CMD",
                        smach_ros.SimpleActionState(
                            "/simon_cmd",
                            SimonCmdAction,
                            goal_cb=_goal_cb,
                            input_keys=["left_action", "right_action", "simon_says"],
                            exec_timeout=rospy.Duration(20.0),
                        ),
                    )
                    smach.Concurrence.add("POSE", WaitForPoseWithPause())

                # Run the mini-container, sharing the parent userdata
                outcome = cc.execute(parent_ud=ud)
                return outcome

        # ─────────── ANNOUNCE / CMD / POSE (dynamic) ────────────
        smach.StateMachine.add(
            "ANNOUNCE",
            DynamicAnnounceCmdDetect(prompt_pub, turn_pub, controller.tts),
            transitions={
                "matched":   "EVALUATE",
                "timeout":   "FAIL",
                "preempted": "FAIL",
                "aborted":   "FAIL",
            },
        )
            

        # ────────────────────────────────────────────────────────────────────
        # Main ANNOUNCE + CMD + POSE concurrence
        # ────────────────────────────────────────────────────────────────────
        # announce_cmd_and_detect = smach.Concurrence(
        #     outcomes=["matched", "timeout", "preempted", "aborted"],
        #     default_outcome="timeout",
        #     outcome_map={
        #         "matched": {"POSE":"matched"},
        #         "timeout": {"POSE":"timeout"},
        #         "preempted": {"POSE":"preempted"},
        #         "aborted": {"TALK":"aborted", "CMD":"aborted"}
        #     },
        #     input_keys=["left_action", "right_action", "simon_says", "turn_idx", "turn_timeout", "success_threshold","pose_matched"],
        #     output_keys=["pose_matched"],
        #     child_termination_cb=lambda outcome_map:outcome_map.get("POSE") in ("matched","timeout")
        # )

        # with announce_cmd_and_detect:
        #     smach.Concurrence.add("TALK", Announce(prompt_pub, turn_pub, controller.tts))
        #     smach.Concurrence.add("CMD", smach_ros.SimpleActionState(
        #         "/simon_cmd", SimonCmdAction,
        #         goal_cb=_goal_cb,
        #         input_keys=["left_action", "right_action", "simon_says"],
        #         exec_timeout=rospy.Duration(10.0)
        #     ))
        #     smach.Concurrence.add("POSE", WaitForPoseWithPause())

        # smach.StateMachine.add("ANNOUNCE", announce_cmd_and_detect,
        #     transitions={
        #         "matched": "EVALUATE",
        #         "timeout": "FAIL",
        #         "preempted": "FAIL",
        #         "aborted": "FAIL"
        #     })



        # Create PauseWaitState instance
        pause_state = PauseWaitState(controller)

        class WaitForPoseWithPause(WaitForPose):
            def execute(self, ud):
                outcome = super().execute(ud)
                if outcome == "matched" and controller.pause_pending:
                    return "PAUSE_AFTER_WAIT"
                return outcome

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

        # smach.StateMachine.add("REWARD", PublishEmotionState(Emotion.HAPPY), transitions={"done":"NEXT_TURN"})
        # smach.StateMachine.add("FAIL", PublishEmotionState(Emotion.SAD), transitions={"done":"NEXT_TURN"})

        # ------------------------------------------------------------------
        # 4-choice phrase lists (add / adjust as you like)
        reward_lines = [
            "Great job!", 
            "Nice one!", 
            "Well done!", 
            ""                    # ← silent variant
        ]

        smach.StateMachine.add(
            "REWARD",
            FeedbackState(Emotion.HAPPY, reward_lines, controller.tts),
            transitions={"done": "RETURN_HOME"}
        )
        smach.StateMachine.add(
            "FAIL",
            FailFeedbackState(controller.tts),
            transitions={"done": "RETURN_HOME"}
        )

        # ⏪ After either REWARD or FAIL, go home before NEXT_TURN
        smach.StateMachine.add(
            "RETURN_HOME",
            ReturnHomeState(controller.cmd_client),
            transitions={"done": "NEXT_TURN"}
        )

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
            "success_threshold": rospy.get_param("~threshold"),
            "static_threshold": rospy.get_param("~static_threshold"),
            "static_start_hold_seconds": rospy.get_param("~static_start_hold_seconds", 1.5),
        }

        # ── status publisher so GUI can enable “Start” when we’re ready ──
        self.status_pub = rospy.Publisher(
            "/simon_game/status", String, queue_size=1, latch=True
        )
        self.ui_state_pub = rospy.Publisher(
            "/simon_game/ui_state", String, queue_size=1, latch=True
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

        # Face expression publisher (latched so the GUI sees the last value)
        self.emotion_pub = rospy.Publisher(
            "/emotion", Emotion, queue_size=1, latch=True)
        # ── Set neutral face immediately 
        self.emotion_pub.publish(Emotion(state=Emotion.NEUTRAL))

        self.cmd_client = SimpleActionClient("/simon_cmd", SimonCmdAction)
        self.cmd_client.wait_for_server()

        # Build state machine, passing in our fixed sequence
        self.sm = build_sm(self.sequence, self.params, self.score_pub, self.prompt_pub, self)
        # Introspection for viz
        self.sis = smach_ros.IntrospectionServer("game_sm", self.sm, "/GAME_SM")
        # Control subscriber
        self.control_sub = rospy.Subscriber('/simon_game/control', String, self.control_cb)
        self.running = False
        # state for optional pre-game steps
        self.intro_done = False
        # Flags for pause/resume 
        self.pause_pending = False
        self.resume_pending = False
        # intro-thread state flags
        self.intro_in_progress = False
        self.pre_game_cancel = threading.Event()
        # action-client for intro waving

        # Start the introspection server
        self.game_thread = None            # keep a handle so we can join()

        # self.cmd_client = SimpleActionClient("/simon_cmd", SimonCmdAction)
        # self.cmd_client.wait_for_server()


    # ─────────────────────────────── PRE-GAME HELPERS ────────────────────────
    def _publish_ui_state(self, ui_state: str):
        self.ui_state_pub.publish(ui_state)

    def _set_ready_state(self, prompt_text: str | None = None):
        if prompt_text:
            self.prompt_pub.publish(prompt_text)
        self._publish_ui_state("ready")
        self.status_pub.publish(READY_STATUS)

    def _rebuild_game_state_machine(self):
        self.sequence = []
        for _ in range(self.params["total_rounds"]):
            l, r = _pick_actions(self.action_pool)
            s = (random.random() < self.params["simon_ratio"])
            self.sequence.append((l, r, s))

        self.sm = build_sm(
            self.sequence, self.params, self.score_pub, self.prompt_pub, self
        )
        self.sis = smach_ros.IntrospectionServer("game_sm", self.sm, "/GAME_SM")

    def _start_game(self):
        if self.running or self.intro_in_progress:
            return
        self.intro_in_progress = True
        threading.Thread(target=self._run_start_game_sequence, daemon=True).start()

    def _run_start_game_sequence(self):
        try:
            self._publish_ui_state("setup_static_pose")
            self.status_pub.publish("Checking static pose before game start.")
            self.pre_game_cancel.clear()

            static_stage = StaticPoseStartStage(
                self.tts,
                self.prompt_pub,
                self.params["static_threshold"],
                self.params["static_start_hold_seconds"],
                self.pre_game_cancel,
            )
            if not static_stage.run():
                self._set_ready_state("Start canceled. Choose the next step.")
                return

            self.running = True
            self.intro_done = True
            self._publish_ui_state("in_game")
            self.status_pub.publish("Game running.")
            self.sis.start()
            self.game_thread = threading.Thread(target=self.run_game, daemon=True)
            self.game_thread.start()
        finally:
            self.intro_in_progress = False

    def _run_instruction_sequence(self) -> bool:
        self._publish_ui_state("setup_instructions")
        self.status_pub.publish("Reading instructions.")
        self.pre_game_cancel.clear()
        rospy.loginfo("[INTRO] Speaking welcome speech")
        self.tts.speak("Hi there!")
        tts_thread = threading.Thread(
            target=self.tts.speak, args=(WELCOME_SPEECH,)
        )
        tts_thread.start()

        goal = SimonCmdGoal(
            gesture_name="D_WAVE_left|D_WAVE_right", simon_says=True
        )
        rospy.loginfo("[INTRO] Dual-arm wave")
        self.cmd_client.send_goal(goal)

        self.cmd_client.wait_for_result()
        if self.pre_game_cancel.is_set():
            self._set_ready_state("Setup canceled. Choose the next step.")
            return False
        tts_thread.join()
        if self.pre_game_cancel.is_set():
            self._set_ready_state("Setup canceled. Choose the next step.")
            return False

        rospy.loginfo("[INTRO] Publishing rules text")
        self.prompt_pub.publish(RULES_TEXT)
        rospy.loginfo("[INTRO] Speaking rules speech")
        self.tts.speak(RULES_SPEECH)
        if self.pre_game_cancel.is_set():
            self._set_ready_state("Setup canceled. Choose the next step.")
            return False
        return True

    def _run_calibration_sequence(self) -> bool:
        self._publish_ui_state("setup_calibration")
        self.status_pub.publish("Calibrating camera.")
        self.pre_game_cancel.clear()
        calib = CalibrationStage(self.tts, self.prompt_pub, self.pre_game_cancel)
        finished = calib.run()
        if not finished:
            self._set_ready_state("Calibration canceled. Choose the next step.")
            return False
        self.prompt_pub.publish("Calibration complete. You can start the game.")
        self.tts.speak("Calibration complete. You can start the game when you are ready.")
        return True

    def _run_instructions(self):
        try:
            if not self._run_instruction_sequence():
                return
            self._set_ready_state(
                "Instructions complete. Calibrate the camera or start the game."
            )
        finally:
            self.intro_in_progress = False

    def _run_calibration(self):
        try:
            if not self._run_calibration_sequence():
                return
            self._set_ready_state()
        finally:
            self.intro_in_progress = False

    def _run_full_setup(self):
        try:
            if not self._run_instruction_sequence():
                return
            if not self._run_calibration_sequence():
                return
            self._set_ready_state("Setup complete. Start the game when you are ready.")
        finally:
            self.intro_in_progress = False

    # ───────────────────────────────── CONTROL CALLBACK ─────────────────────
    def control_cb(self, msg: String):
        cmd = msg.data
        rospy.loginfo(f"Received control: {cmd}")

        if cmd == "start":
            cmd = "read_instructions"
        elif cmd == "continue":
            cmd = "resume" if self.running else "start_game"

        if cmd == "read_instructions" and not self.running and not self.intro_in_progress:
            self.intro_in_progress = True
            threading.Thread(target=self._run_instructions, daemon=True).start()
            return
        if cmd == "run_full_setup" and not self.running and not self.intro_in_progress:
            self.intro_in_progress = True
            threading.Thread(target=self._run_full_setup, daemon=True).start()
            return
        if cmd == "calibrate_camera" and not self.running and not self.intro_in_progress:
            self.intro_in_progress = True
            threading.Thread(target=self._run_calibration, daemon=True).start()
            return
        if cmd == "start_game" and not self.running and not self.intro_in_progress:
            self._start_game()
            return
        if self.intro_in_progress:
            if cmd in ("stop", "restart"):
                rospy.loginfo("[game_runner] Canceling active pre-game step")
                self.pre_game_cancel.set()
                if cmd == "restart":
                    self.score_pub.publish(0)
                    rospy.Publisher('/simon_game/turn_id', Int32, queue_size=1, latch=True).publish(0)
                self._set_ready_state("Setup canceled. Choose the next step.")
            elif cmd == "quit":
                rospy.signal_shutdown('Quit via GUI')
            rospy.loginfo("[game_runner] Ignoring command while a pre-game step is running")
            return

        # ----- post-intro: normal game controls -----
        if cmd == 'pause' and self.running:
            self.pause_pending = True
            self.sm.request_preempt()
            self._publish_ui_state("paused")
            self.status_pub.publish("Game paused.")
        elif cmd == 'resume':
            self.resume_pending = True
            self.pause_pending = False
            rospy.loginfo("[game_runner] Resume requested – continuing game")
            self._publish_ui_state("in_game")
            self.status_pub.publish("Game running.")
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

            # 2) Clear intro handshake so the next run can choose any entry point
            self.intro_done = False
            self.intro_in_progress = False

            # 3) Publish reset values so the GUI shows   Turn: 0 / Score: 0
            self.score_pub.publish(0)
            rospy.Publisher('/simon_game/turn_id', Int32, queue_size=1, latch=True).publish(0)

            # 4) Build a brand-new gesture sequence and SMACH graph
            self._rebuild_game_state_machine()
            rospy.loginfo("[game_runner] Ready for new Start")

            # tell GUI it’s safe to re-enable the stage buttons
            self._set_ready_state("Session reset. Choose the next step.")

            return
        elif cmd == 'quit':
            rospy.signal_shutdown('Quit via GUI')

    # ───────────────────────────────── GAME LOOP ────────────────────────────
    def run_game(self):
        outcome = self.sm.execute()
        final_score = self.sm.userdata.score
        total      = self.params["total_rounds"]
        game_over_text = f"Game Over! Your final score is {final_score} out of {total}."
        self._publish_ui_state("game_over")
        self.status_pub.publish(game_over_text)
        self.prompt_pub.publish(game_over_text)
        self.tts.speak(game_over_text)
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
    controller.ui_state_pub.publish("ready")
    controller.status_pub.publish(READY_STATUS)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
