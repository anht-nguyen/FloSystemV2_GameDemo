#!/usr/bin/env python3
"""
ROS GUI for Dual‑Arm Simon Says – *Qt State Machine Edition*
===========================================================
Re‑implements the control panel using **QStateMachine** so that GUI behaviour
faithfully mirrors the state / event flow defined below. All GUI element
properties (enabled / disabled states, timers, counters) are updated through
state‑entry callbacks rather than ad‑hoc logic sprinkled across the code.

| State        | Event (button)   | Action published to /simon_game/control | Next GUI State |
|--------------|------------------|-----------------------------------------|----------------|
| Idle         | Start            | start ‒ reset intro flags, show rules   | Intro          |
| Intro        | Prompt received  | ‒ display rules; enable Continue/Stop   | Intro_Wait     |
| Intro_Wait   | Continue         | continue ‒ begin countdown & game loop  | InGame         |
| Intro_Wait   | Stop             | stop ‒ abort intro                      | GameOver       |
| InGame       | Prompt received  | ‒ update action prompt / turn / timer   | InGame         |
| InGame       | Pause            | pause ‒ freeze game & timer             | Paused         |
| InGame       | Stop             | stop ‒ terminate game                   | GameOver       |
| Paused       | Continue         | continue ‒ resume game & timer          | InGame         |
| Paused       | Stop             | stop ‒ abandon game                     | GameOver       |
| GameOver     | Restart          | restart ‒ full reset; re-enable Start   | Idle           |
| Any state    | Quit             | quit ‒ shutdown GUI & ROS node          | Terminated     |

"""
from __future__ import annotations

import sys
from enum import Enum

import rospy
from PyQt5.QtCore import Qt, QTimer, QState, QStateMachine, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from std_msgs.msg import Int32, String


class GameState(str, Enum):
    IDLE = "Idle"
    INTRO = "Intro"
    INTRO_WAIT = "Intro_Wait"
    IN_GAME = "InGame"
    PAUSED = "Paused"
    GAME_OVER = "GameOver"
    TERMINATED = "Terminated"


class SimonGUI(QWidget):
    """Main GUI widget driven by a Qt   StateMachine."""

    # ──────────────── *FSM trigger signals* ────────────────
    startClicked = pyqtSignal()
    continueClicked = pyqtSignal()
    pauseClicked = pyqtSignal()
    stopClicked = pyqtSignal()
    restartClicked = pyqtSignal()
    quitClicked = pyqtSignal()
    rulesReceived = pyqtSignal()  # emitted when rules prompt arrives

    # ───────────────────────── GUI lifecycle ──────────────────────────

    def __init__(self):
        super().__init__()

        # ── ROS init ────────────────────────────────────────────────
        rospy.init_node("simon_gui", anonymous=True)
        self.cmd_pub = rospy.Publisher("/simon_game/control", String, queue_size=10)
        rospy.Subscriber("/simon_game/score", Int32, self._cb_score)
        rospy.Subscriber("/simon_game/prompt", String, self._cb_prompt)
        rospy.Subscriber("/simon_game/turn_id", Int32, self._cb_turn)
        rospy.Subscriber("/simon_game/status", String, self._cb_status)

        # ── params ──────────────────────────────────────────────────
        self.turn_timeout = int(rospy.get_param("~turn_timeout", 4))
        self.total_rounds = int(rospy.get_param("~total_rounds", 15))

        # ── internal counters ───────────────────────────────────────
        self.current_turn = 0
        self.current_score = 0
        self.remaining_time = self.turn_timeout
        self.current_state: GameState = GameState.IDLE
        self.ready_for_start = False        # ← only true after “Waiting…” seen

        # ── GUI widgets ─────────────────────────────────────────────
        self._build_ui()

        # ── timer for countdown ─────────────────────────────────────
        self.ticker = QTimer(self)
        self.ticker.timeout.connect(self._tick_timer)

        # ── finite‑state machine ────────────────────────────────────
        self._build_fsm()

        # ── ensure buttons start DISABLED before any ROS status ─────
        #   (Qt queues the state-entry signal, so without this there’s
        #    a brief moment where “Start” appears enabled.)
        self._update_buttons()

    # ─────────────────────────── UI set‑up ───────────────────────────

    def _build_ui(self):
        self.setWindowTitle("Dual‑Arm Simon Says – Control Panel")
        self.setFixedSize(380, 420)

        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignTop)
        self.setLayout(layout)

        # prompt / rules label -------------------------------------------------
        self.lbl_prompt = QLabel("Prompt: –")
        self.lbl_prompt.setWordWrap(True)
        self.lbl_prompt.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        layout.addWidget(self.lbl_prompt)

        # turn + score row -----------------------------------------------------
        self.lbl_turn = QLabel(f"Turn: 0/{self.total_rounds}")
        self.lbl_score = QLabel("Score: 0")
        for l in (self.lbl_turn, self.lbl_score):
            l.setAlignment(Qt.AlignCenter)
            l.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        row = QHBoxLayout()
        row.addWidget(self.lbl_turn)
        row.addWidget(self.lbl_score)
        layout.addLayout(row)

        # countdown ------------------------------------------------------------
        self.lbl_timer = QLabel(self._fmt_time(self.remaining_time))
        self.lbl_timer.setAlignment(Qt.AlignCenter)
        self.lbl_timer.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        layout.addWidget(self.lbl_timer)

        # status ---------------------------------------------------------------
        self.lbl_status = QLabel(f"Status: {self.current_state.value}")
        self.lbl_status.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        layout.addWidget(self.lbl_status)

        # control buttons ------------------------------------------------------
        self.btn_start = QPushButton("Start")
        self.btn_continue = QPushButton("Continue")
        self.btn_pause = QPushButton("Pause")
        self.btn_stop = QPushButton("Stop")
        self.btn_restart = QPushButton("Restart")
        self.btn_quit = QPushButton("Quit")

        # button → command mapping --------------------------------------------
        self._cmd_map = {
            self.btn_start: ("start", self.startClicked),
            self.btn_continue: ("continue", self.continueClicked),
            self.btn_pause: ("pause", self.pauseClicked),
            self.btn_stop: ("stop", self.stopClicked),
            self.btn_restart: ("restart", self.restartClicked),
            self.btn_quit: ("quit", self.quitClicked),
        }

        row1 = QHBoxLayout()
        for b in (self.btn_start, self.btn_continue, self.btn_pause):
            b.clicked.connect(self._on_button)
            row1.addWidget(b)
        layout.addLayout(row1)

        row2 = QHBoxLayout()
        for b in (self.btn_stop, self.btn_restart, self.btn_quit):
            b.clicked.connect(self._on_button)
            row2.addWidget(b)
        layout.addLayout(row2)

    # ───────────────────── build the Qt state machine ─────────────────────

    def _build_fsm(self):
        self.machine = QStateMachine(self)

        # create states --------------------------------------------------------
        self.s_idle = QState()
        self.s_intro = QState()
        self.s_intro_wait = QState()
        self.s_in_game = QState()
        self.s_paused = QState()
        self.s_game_over = QState()
        self.s_terminated = QState()

        self.machine.addState(self.s_idle)
        self.machine.addState(self.s_intro)
        self.machine.addState(self.s_intro_wait)
        self.machine.addState(self.s_in_game)
        self.machine.addState(self.s_paused)
        self.machine.addState(self.s_game_over)
        self.machine.addState(self.s_terminated)
        self.machine.setInitialState(self.s_idle)

        # ── transitions -------------------------------------------------------
        self.s_idle.addTransition(self.startClicked, self.s_intro)

        self.s_intro.addTransition(self.rulesReceived, self.s_intro_wait)

        self.s_intro_wait.addTransition(self.continueClicked, self.s_in_game)
        self.s_intro_wait.addTransition(self.stopClicked, self.s_game_over)

        self.s_in_game.addTransition(self.pauseClicked, self.s_paused)
        self.s_in_game.addTransition(self.stopClicked, self.s_game_over) # Stop ⇒ Game Over
        self.s_in_game.addTransition(self.quitClicked, self.s_terminated)
        # self.s_in_game.addTransition(self.restartClicked, self.s_idle)

        self.s_paused.addTransition(self.continueClicked, self.s_in_game)
        self.s_paused.addTransition(self.stopClicked, self.s_game_over)
        # self.s_paused.addTransition(self.restartClicked, self.s_idle)

        # Game Over ⇒ Restart ⇒ Idle
        self.s_game_over.addTransition(self.restartClicked, self.s_idle)

        # global transitions ---------------------------------------------------
        for st in (self.s_idle, self.s_intro, self.s_intro_wait, self.s_in_game, self.s_paused, self.s_game_over):
            st.addTransition(self.quitClicked, self.s_terminated)

        # ── state‑entry callbacks --------------------------------------------
        self.s_idle.entered.connect(lambda: self._on_enter_state(GameState.IDLE))
        self.s_intro.entered.connect(lambda: self._on_enter_state(GameState.INTRO))
        self.s_intro_wait.entered.connect(lambda: self._on_enter_state(GameState.INTRO_WAIT))
        self.s_in_game.entered.connect(lambda: self._on_enter_state(GameState.IN_GAME))
        self.s_paused.entered.connect(lambda: self._on_enter_state(GameState.PAUSED))
        self.s_game_over.entered.connect(lambda: self._on_enter_state(GameState.GAME_OVER)) 
        self.s_terminated.entered.connect(lambda: self._on_enter_state(GameState.TERMINATED))

        self.machine.start()

    # ─────────────────────── button / ROS handlers ────────────────────────

    def _on_button(self):
        btn = self.sender()
        cmd, signal = self._cmd_map[btn]
        # publish ROS control command ----------------------------------------
        self.cmd_pub.publish(cmd)
        rospy.loginfo(f"[GUI] Sent command: {cmd}")
        # emit FSM trigger ----------------------------------------------------
        signal.emit()
        if cmd == "quit":
            # terminate ROS + Qt event loops
            rospy.signal_shutdown("Quit via GUI")
            QApplication.quit()

    # ── state entry common logic --------------------------------------------

    def _on_enter_state(self, new_state: GameState):
        self.current_state = new_state
        self.lbl_status.setText(f"Status: {new_state.value}")
        rospy.loginfo(f"[GUI] → {new_state.value}")

        # manage countdown ---------------------------------------------------
        if new_state == GameState.IN_GAME:
            self.remaining_time = self.turn_timeout
            self.lbl_timer.setText(self._fmt_time(self.remaining_time))
            if not self.ticker.isActive():
                self.ticker.start(1000)
        else:
            self.ticker.stop()
            self.remaining_time = self.turn_timeout
            self.lbl_timer.setText(self._fmt_time(self.remaining_time))

        # reset counters at fresh intro --------------------------------------
        if new_state == GameState.INTRO:
            self.current_turn = 0
            self.current_score = 0
            self.lbl_turn.setText(f"Turn: 0/{self.total_rounds}")
            self.lbl_score.setText("Score: 0")
            self.lbl_prompt.setText("Welcome to Simon Says game with Flo robot!\n") 
        elif new_state == GameState.IDLE:
            # Coming here after Stop/Restart → wait for fresh “Waiting…”
            self.ready_for_start = False
            # clear GUI after stop/restart
            self.lbl_prompt.setText("Press Start to begin\n")
        elif new_state == GameState.GAME_OVER:
            # Stop countdown, keep the last score/turn visible
            self.ticker.stop()
            self.remaining_time = self.turn_timeout
            self.lbl_timer.setText(self._fmt_time(self.remaining_time))

        self._update_buttons()

    # ───────────────────────── countdown tick ──────────────────────────────

    def _tick_timer(self):
        if self.remaining_time > 0:
            self.remaining_time -= 1
            self.lbl_timer.setText(self._fmt_time(self.remaining_time))
        else:
            self.ticker.stop()

    # ─────────────────────────── ROS callbacks ─────────────────────────────

    def _cb_score(self, msg: Int32):
        self.current_score = msg.data
        self.lbl_score.setText(f"Score: {self.current_score}")

    def _cb_prompt(self, msg: String):
        text = msg.data
        self.lbl_prompt.setText(f"Prompt: {text}")
        # intro → intro_wait trigger ----------------------------------------
        if self.current_state == GameState.INTRO:
            self.rulesReceived.emit()

    def _cb_turn(self, msg: Int32):
        self.current_turn = msg.data
        self.lbl_turn.setText(f"Turn: {self.current_turn}/{self.total_rounds}")
        if self.current_state == GameState.IN_GAME:
            self.remaining_time = self.turn_timeout
            self.lbl_timer.setText(self._fmt_time(self.remaining_time))

    # ───────────────────────── helper methods ─────────────────────────────

    @staticmethod
    def _fmt_time(seconds: int) -> str:
        m, s = divmod(seconds, 60)
        return f"{m:02d}:{s:02d}"

   # ───────────────── status/ready callback ────────────────────────
    def _cb_status(self, msg: String):
       if "Waiting for Start command" in msg.data:
           self.ready_for_start = True
           self._update_buttons()          # re-evaluate button states


    def _update_buttons(self):
        # Enable/disable buttons based on current state -------------------
        # disable all first ---------------------------------------------------
        for b in (
            self.btn_start,
            self.btn_continue,
            self.btn_pause,
            self.btn_stop,
            self.btn_restart,
            self.btn_quit,
        ):
            b.setEnabled(False)

        st = self.current_state
        if st == GameState.IDLE and self.ready_for_start:
            self.btn_start.setEnabled(True)
        elif st == GameState.INTRO:
            pass  # waiting for rules
        elif st == GameState.INTRO_WAIT:
            self.btn_continue.setEnabled(True)
            self.btn_stop.setEnabled(True)
            # self.btn_restart.setEnabled(True)
        elif st == GameState.IN_GAME:
            self.btn_pause.setEnabled(True)
            self.btn_stop.setEnabled(True)
            # self.btn_restart.setEnabled(True)
        elif st == GameState.PAUSED:
            self.btn_continue.setEnabled(True)
            self.btn_stop.setEnabled(True)
            # self.btn_restart.setEnabled(True)
        elif st == GameState.GAME_OVER:          # ← NEW
            self.btn_restart.setEnabled(True)

        # Quit always enabled -------------------------------------------------
        self.btn_quit.setEnabled(True)


# ────────────────────────────── main ───────────────────────────────────────

def main():
    app = QApplication(sys.argv)
    gui = SimonGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
