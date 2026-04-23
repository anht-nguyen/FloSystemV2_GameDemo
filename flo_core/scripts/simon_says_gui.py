#!/usr/bin/env python3
"""
ROS GUI for Dual‑Arm Simon Says – merged camera + controller window.
"""
from __future__ import annotations

import signal
import subprocess
import sys
from enum import Enum

import rospy
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtCore import Qt, QState, QStateMachine, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPalette, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QStyle,
    QStyleOptionButton,
    QStylePainter,
    QVBoxLayout,
    QWidget,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String


class GameState(str, Enum):
    IDLE = "Idle"
    INTRO = "Intro"
    INTRO_WAIT = "Intro_Wait"
    CALIBRATING = "Calibrating"
    READY = "Ready"
    IN_GAME = "InGame"
    PAUSED = "Paused"
    GAME_OVER = "GameOver"
    TERMINATED = "Terminated"


class CameraView(QWidget):
    """Paint camera frames into a stable viewport instead of QLabel pixmap layout."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._pixmap = None
        self._placeholder = "Waiting for tracker preview..."
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(320, 240)

    def set_frame(self, pixmap: QPixmap):
        self._pixmap = pixmap
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)

        content_rect = self.rect().adjusted(24, 24, -24, -24)
        painter.fillRect(self.rect(), QColor("#060b10"))

        if self._pixmap is None or self._pixmap.isNull():
            painter.setPen(QColor("#89a1ba"))
            painter.drawText(content_rect, Qt.AlignCenter | Qt.TextWordWrap, self._placeholder)
            return

        scaled = self._pixmap.scaled(
            content_rect.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        x_pos = content_rect.x() + (content_rect.width() - scaled.width()) // 2
        y_pos = content_rect.y() + (content_rect.height() - scaled.height()) // 2
        painter.drawPixmap(x_pos, y_pos, scaled)


class SetupActionButton(QPushButton):
    """Button with a bold title and lighter subtitle."""

    def __init__(self, title: str, subtitle: str, parent=None):
        super().__init__("", parent)
        self._title = title
        self._subtitle = subtitle
        self.setMinimumHeight(58)

    def paintEvent(self, event):
        painter = QStylePainter(self)
        option = QStyleOptionButton()
        self.initStyleOption(option)
        option.text = ""
        painter.drawControl(QStyle.CE_PushButton, option)

        content_rect = self.style().subElementRect(
            QStyle.SE_PushButtonContents,
            option,
            self,
        )
        color_group = QPalette.Active if self.isEnabled() else QPalette.Disabled
        text_color = self.palette().color(color_group, QPalette.ButtonText)

        title_font = QFont(self.font())
        title_font.setBold(True)
        title_font.setPointSize(max(title_font.pointSize(), 16))

        subtitle_font = QFont(self.font())
        subtitle_font.setBold(False)
        subtitle_font.setPointSize(max(subtitle_font.pointSize() - 2, 11))

        painter.setFont(title_font)
        title_height = painter.fontMetrics().height()
        painter.setFont(subtitle_font)
        subtitle_height = painter.fontMetrics().height()
        spacing = 2
        total_height = title_height + spacing + subtitle_height
        start_y = content_rect.y() + (content_rect.height() - total_height) // 2

        painter.setPen(text_color)
        painter.setFont(title_font)
        painter.drawText(
            content_rect.adjusted(0, start_y - content_rect.y(), 0, 0),
            Qt.AlignHCenter | Qt.AlignTop,
            self._title,
        )
        painter.setFont(subtitle_font)
        painter.drawText(
            content_rect.adjusted(0, start_y + title_height + spacing - content_rect.y(), 0, 0),
            Qt.AlignHCenter | Qt.AlignTop,
            self._subtitle,
        )


class SimonGUI(QWidget):
    """Main GUI widget driven by a Qt StateMachine."""

    fullSetupClicked = pyqtSignal()
    instructionsClicked = pyqtSignal()
    calibrateClicked = pyqtSignal()
    startGameClicked = pyqtSignal()
    conversationToggleClicked = pyqtSignal()
    pauseResumeClicked = pyqtSignal()
    pauseClicked = pyqtSignal()
    resumeClicked = pyqtSignal()
    stopClicked = pyqtSignal()
    restartClicked = pyqtSignal()
    quitClicked = pyqtSignal()
    rulesReceived = pyqtSignal()
    calibrationStarted = pyqtSignal()
    calibrationFinished = pyqtSignal()
    cameraFrameReady = pyqtSignal(QImage)
    scoreUpdated = pyqtSignal(int)
    promptUpdated = pyqtSignal(str)
    turnUpdated = pyqtSignal(int)
    statusUpdated = pyqtSignal(str)
    uiStateUpdated = pyqtSignal(str)
    conversationTextReceived = pyqtSignal(str, str)
    backendGameOver = pyqtSignal()

    def __init__(self):
        super().__init__()

        rospy.init_node("simon_gui", anonymous=True)
        self.cmd_pub = rospy.Publisher("/simon_game/control", String, queue_size=10)
        self.turn_timeout = int(rospy.get_param("/game_runner/turn_timeout"))
        self.total_rounds = int(rospy.get_param("/game_runner/total_rounds"))
        self.min_rounds = 1

        self.current_turn = 0
        self.current_score = 0
        self.remaining_time = self.turn_timeout
        self.current_state: GameState = GameState.IDLE
        self.ready_for_start = False
        self.backend_ui_state = "booting"
        self.conversation_process: subprocess.Popen | None = None
        self.bridge = CvBridge()
        self.current_camera_pixmap = None

        self._build_ui()
        self.cameraFrameReady.connect(self._update_camera_frame)
        self.scoreUpdated.connect(self._apply_score_update)
        self.promptUpdated.connect(self._apply_prompt_update)
        self.turnUpdated.connect(self._apply_turn_update)
        self.statusUpdated.connect(self._apply_status_update)
        self.uiStateUpdated.connect(self._apply_ui_state_update)
        self.conversationTextReceived.connect(self._append_conversation_log)

        self.ticker = QTimer(self)
        self.ticker.timeout.connect(self._tick_timer)
        self.conversation_monitor = QTimer(self)
        self.conversation_monitor.timeout.connect(self._sync_conversation_process)
        self.conversation_monitor.start(1000)

        self._build_fsm()
        self._update_buttons()

        # Subscribe after UI/state setup so early messages cannot hit half-built objects.
        rospy.Subscriber("/simon_game/score", Int32, self._cb_score)
        rospy.Subscriber("/simon_game/prompt", String, self._cb_prompt)
        rospy.Subscriber("/simon_game/turn_id", Int32, self._cb_turn)
        rospy.Subscriber("/simon_game/status", String, self._cb_status)
        rospy.Subscriber("/simon_game/ui_state", String, self._cb_ui_state)
        rospy.Subscriber("/conversation/user_text", String, self._cb_user_text)
        rospy.Subscriber("/conversation/assistant_text", String, self._cb_assistant_text)
        rospy.Subscriber(
            rospy.get_param("~preview_topic", "/arm_hand_tracker/preview_image"),
            Image,
            self._cb_preview_image,
            queue_size=1,
            buff_size=2 ** 24,
        )

    def _build_ui(self):
        self.setWindowTitle("Dual-Arm Simon Says")
        self.setWindowFlags(Qt.Window)
        self.setStyleSheet(
            """
            QWidget {
                background: #0f1720;
                color: #f4f7fb;
                font-family: "DejaVu Sans";
            }
            QFrame#cameraPanel {
                background: #060b10;
                border: 1px solid #1d2937;
                border-radius: 18px;
            }
            QFrame#controlPanel {
                background: #16212d;
                border: 1px solid #243243;
                border-radius: 18px;
            }
            QLabel#cameraTitle {
                font-size: 22px;
                font-weight: 700;
                color: #d7e4f5;
            }
            QLabel#metric {
                background: #1a2735;
                border: 1px solid #2e4359;
                border-radius: 12px;
                padding: 10px 12px;
                font-size: 17px;
                font-weight: 600;
            }
            QLabel#roundsValue {
                background: #1a2735;
                border: 1px solid #2e4359;
                border-radius: 12px;
                padding: 10px 12px;
                font-size: 20px;
                font-weight: 700;
                min-width: 76px;
            }
            QLabel#promptBox {
                background: #1a2735;
                border: 1px solid #2e4359;
                border-radius: 12px;
                padding: 14px;
                font-size: 18px;
            }
            QLabel#logTitle {
                font-size: 18px;
                font-weight: 700;
                color: #d7e4f5;
                margin-top: 6px;
            }
            QPlainTextEdit#conversationLog {
                background: #0f1822;
                border: 1px solid #2e4359;
                border-radius: 12px;
                padding: 10px;
                font-size: 15px;
            }
            QPushButton {
                background: #2b8a78;
                border: none;
                border-radius: 12px;
                color: white;
                font-size: 16px;
                font-weight: 700;
                min-height: 42px;
                padding: 8px 12px;
            }
            QPushButton:disabled {
                background: #4a5d70;
                color: #aab7c3;
            }
            QPushButton#danger {
                background: #b54141;
            }
            QPushButton#secondary {
                background: #405f8d;
            }
            QPushButton#conversationToggle {
                background: #2f6db2;
            }
            QPushButton#conversationToggle[running="true"] {
                background: #c45a2d;
            }
            QPushButton#roundAdjust {
                background: #35506f;
                min-height: 38px;
                min-width: 52px;
                padding: 6px 10px;
            }
            """
        )

        root = QHBoxLayout()
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(18)
        self.setLayout(root)

        camera_panel = QFrame()
        camera_panel.setObjectName("cameraPanel")
        camera_layout = QVBoxLayout(camera_panel)
        camera_layout.setContentsMargins(18, 18, 18, 18)
        camera_layout.setSpacing(12)

        camera_title = QLabel("Camera View")
        camera_title.setObjectName("cameraTitle")
        camera_layout.addWidget(camera_title)

        self.camera_view = CameraView()
        self.camera_view.setStyleSheet(
            "background: #060b10; border: 1px dashed #395169; border-radius: 14px;"
        )
        camera_layout.addWidget(self.camera_view, 1)

        root.addWidget(camera_panel, 3)

        control_panel = QFrame()
        control_panel.setObjectName("controlPanel")
        control_layout = QVBoxLayout(control_panel)
        control_layout.setContentsMargins(18, 18, 18, 18)
        control_layout.setSpacing(14)

        controller_section = QWidget()
        controller_layout = QVBoxLayout(controller_section)
        controller_layout.setContentsMargins(0, 0, 0, 0)
        controller_layout.setSpacing(14)
        controller_layout.setAlignment(Qt.AlignTop)

        title = QLabel("Game Controller")
        title.setStyleSheet("font-size: 24px; font-weight: 700;")
        controller_layout.addWidget(title)

        self.lbl_prompt = QLabel("Prompt: –")
        self.lbl_prompt.setObjectName("promptBox")
        self.lbl_prompt.setWordWrap(True)
        controller_layout.addWidget(self.lbl_prompt)

        self.lbl_turn = QLabel(f"Turn: 0/{self.total_rounds}")
        self.lbl_score = QLabel("Score: 0")
        for widget in (self.lbl_turn, self.lbl_score):
            widget.setAlignment(Qt.AlignCenter)
            widget.setObjectName("metric")
        row = QHBoxLayout()
        row.addWidget(self.lbl_turn)
        row.addWidget(self.lbl_score)
        controller_layout.addLayout(row)

        self.lbl_timer = QLabel(self._fmt_time(self.remaining_time))
        self.lbl_timer.setAlignment(Qt.AlignCenter)
        self.lbl_timer.setObjectName("metric")
        controller_layout.addWidget(self.lbl_timer)

        self.lbl_status = QLabel(f"Status: {self.current_state.value}")
        self.lbl_status.setObjectName("metric")
        controller_layout.addWidget(self.lbl_status)

        rounds_row = QHBoxLayout()
        rounds_row.setSpacing(10)
        rounds_label = QLabel("Total Rounds")
        rounds_label.setStyleSheet("font-size: 18px; font-weight: 700; color: #d7e4f5;")
        self.btn_rounds_down = QPushButton("▼")
        self.btn_rounds_up = QPushButton("▲")
        self.btn_rounds_down.setObjectName("roundAdjust")
        self.btn_rounds_up.setObjectName("roundAdjust")
        self.btn_rounds_value = QLabel()
        self.btn_rounds_value.setObjectName("roundsValue")
        self.btn_rounds_value.setAlignment(Qt.AlignCenter)
        self.btn_rounds_down.clicked.connect(lambda: self._adjust_total_rounds(-1))
        self.btn_rounds_up.clicked.connect(lambda: self._adjust_total_rounds(1))
        rounds_row.addWidget(rounds_label)
        rounds_row.addStretch(1)
        rounds_row.addWidget(self.btn_rounds_down)
        rounds_row.addWidget(self.btn_rounds_value)
        rounds_row.addWidget(self.btn_rounds_up)
        controller_layout.addLayout(rounds_row)
        self._refresh_rounds_display()

        self.btn_full_setup = SetupActionButton(
            "Run Full Setup",
            "(Instr. + Calib.)",
        )
        self.btn_instructions = QPushButton("Read Instructions")
        self.btn_calibrate = QPushButton("Calibrate Camera")
        self.btn_start_game = QPushButton("Start Game")
        self.btn_conversation = QPushButton("Start Conversation")
        self.btn_conversation.setObjectName("conversationToggle")
        self.btn_pause_resume = QPushButton("Pause Game")
        self.btn_stop = QPushButton("Stop")
        self.btn_restart = QPushButton("Restart Session")
        self.btn_stop.setObjectName("danger")

        self._cmd_map = {
            self.btn_full_setup: ("run_full_setup", self.fullSetupClicked),
            self.btn_instructions: ("read_instructions", self.instructionsClicked),
            self.btn_calibrate: ("calibrate_camera", self.calibrateClicked),
            self.btn_start_game: ("start_game", self.startGameClicked),
            self.btn_pause_resume: ("pause_resume", self.pauseResumeClicked),
            self.btn_stop: ("stop", self.stopClicked),
            self.btn_restart: ("restart", self.restartClicked),
        }

        row1 = QHBoxLayout()
        for button in (self.btn_full_setup, self.btn_instructions):
            button.clicked.connect(self._on_button)
            row1.addWidget(button)
        controller_layout.addLayout(row1)

        row2 = QHBoxLayout()
        for button in (self.btn_calibrate, self.btn_start_game):
            button.clicked.connect(self._on_button)
            row2.addWidget(button)
        controller_layout.addLayout(row2)

        row3 = QHBoxLayout()
        for button in (self.btn_pause_resume, self.btn_stop, self.btn_restart):
            button.clicked.connect(self._on_button)
            row3.addWidget(button)
        controller_layout.addLayout(row3)

        conversation_section = QWidget()
        conversation_layout = QVBoxLayout(conversation_section)
        conversation_layout.setContentsMargins(0, 0, 0, 0)
        conversation_layout.setSpacing(10)

        log_title = QLabel("Conversation Log")
        log_title.setObjectName("logTitle")
        conversation_layout.addWidget(log_title)

        conversation_row = QHBoxLayout()
        self.btn_conversation.clicked.connect(self._on_button)
        conversation_row.addWidget(self.btn_conversation)
        conversation_layout.addLayout(conversation_row)

        self.txt_conversation_log = QPlainTextEdit()
        self.txt_conversation_log.setObjectName("conversationLog")
        self.txt_conversation_log.setReadOnly(True)
        self.txt_conversation_log.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.txt_conversation_log.setPlaceholderText(
            "Robot speech and recognized user speech will appear here."
        )
        self.txt_conversation_log.setMinimumHeight(240)
        conversation_layout.addWidget(self.txt_conversation_log, 1)

        control_layout.addWidget(controller_section, 1)
        control_layout.addWidget(conversation_section, 1)

        root.addWidget(control_panel, 1)

    def _build_fsm(self):
        self.machine = QStateMachine(self)

        self.s_idle = QState()
        self.s_intro = QState()
        self.s_intro_wait = QState()
        self.s_in_game = QState()
        self.s_calibrating = QState()
        self.s_ready = QState()
        self.s_paused = QState()
        self.s_game_over = QState()
        self.s_terminated = QState()

        self.machine.addState(self.s_idle)
        self.machine.addState(self.s_intro)
        self.machine.addState(self.s_intro_wait)
        self.machine.addState(self.s_calibrating)
        self.machine.addState(self.s_ready)
        self.machine.addState(self.s_in_game)
        self.machine.addState(self.s_paused)
        self.machine.addState(self.s_game_over)
        self.machine.addState(self.s_terminated)
        self.machine.setInitialState(self.s_idle)

        self.s_idle.addTransition(self.fullSetupClicked, self.s_intro)
        self.s_idle.addTransition(self.instructionsClicked, self.s_intro)
        self.s_idle.addTransition(self.calibrateClicked, self.s_calibrating)
        self.s_idle.addTransition(self.startGameClicked, self.s_in_game)
        self.s_intro.addTransition(self.rulesReceived, self.s_intro_wait)
        self.s_intro.addTransition(self.calibrationStarted, self.s_calibrating)
        self.s_intro_wait.addTransition(self.instructionsClicked, self.s_intro)
        self.s_intro_wait.addTransition(self.fullSetupClicked, self.s_intro)
        self.s_intro_wait.addTransition(self.calibrateClicked, self.s_calibrating)
        self.s_intro_wait.addTransition(self.startGameClicked, self.s_in_game)
        self.s_intro_wait.addTransition(self.stopClicked, self.s_ready)
        self.s_intro_wait.addTransition(self.calibrationStarted, self.s_calibrating)
        self.s_calibrating.addTransition(self.calibrationFinished, self.s_ready)
        self.s_calibrating.addTransition(self.stopClicked, self.s_ready)
        self.s_ready.addTransition(self.fullSetupClicked, self.s_intro)
        self.s_ready.addTransition(self.instructionsClicked, self.s_intro)
        self.s_ready.addTransition(self.calibrateClicked, self.s_calibrating)
        self.s_ready.addTransition(self.startGameClicked, self.s_in_game)
        self.s_in_game.addTransition(self.pauseClicked, self.s_paused)
        self.s_in_game.addTransition(self.stopClicked, self.s_game_over)
        self.s_paused.addTransition(self.resumeClicked, self.s_in_game)
        self.s_paused.addTransition(self.stopClicked, self.s_game_over)
        self.s_game_over.addTransition(self.restartClicked, self.s_idle)

        for state in (
            self.s_intro,
            self.s_intro_wait,
            self.s_calibrating,
            self.s_ready,
            self.s_in_game,
            self.s_paused,
        ):
            state.addTransition(self.restartClicked, self.s_idle)

        for state in (
            self.s_intro,
            self.s_intro_wait,
            self.s_calibrating,
            self.s_ready,
            self.s_in_game,
            self.s_paused,
        ):
            state.addTransition(self.backendGameOver, self.s_game_over)

        for state in (
            self.s_idle,
            self.s_intro,
            self.s_intro_wait,
            self.s_calibrating,
            self.s_ready,
            self.s_in_game,
            self.s_paused,
            self.s_game_over,
        ):
            state.addTransition(self.quitClicked, self.s_terminated)

        self.s_idle.entered.connect(lambda: self._on_enter_state(GameState.IDLE))
        self.s_intro.entered.connect(lambda: self._on_enter_state(GameState.INTRO))
        self.s_intro_wait.entered.connect(lambda: self._on_enter_state(GameState.INTRO_WAIT))
        self.s_calibrating.entered.connect(lambda: self._on_enter_state(GameState.CALIBRATING))
        self.s_ready.entered.connect(lambda: self._on_enter_state(GameState.READY))
        self.s_in_game.entered.connect(lambda: self._on_enter_state(GameState.IN_GAME))
        self.s_paused.entered.connect(lambda: self._on_enter_state(GameState.PAUSED))
        self.s_game_over.entered.connect(lambda: self._on_enter_state(GameState.GAME_OVER))
        self.s_terminated.entered.connect(lambda: self._on_enter_state(GameState.TERMINATED))

        self.machine.start()

    def _on_button(self):
        btn = self.sender()
        if btn == self.btn_conversation:
            self._toggle_conversation()
            self.conversationToggleClicked.emit()
            return

        cmd, signal = self._cmd_map[btn]
        if cmd == "pause_resume":
            cmd = "resume" if self.current_state == GameState.PAUSED else "pause"
            signal = self.resumeClicked if cmd == "resume" else self.pauseClicked
        self.cmd_pub.publish(cmd)
        rospy.loginfo(f"[GUI] Sent command: {cmd}")
        if cmd in ("run_full_setup", "read_instructions", "calibrate_camera", "start_game", "restart"):
            if cmd != "restart":
                self.ready_for_start = False
            self._update_buttons()
        signal.emit()
        if cmd == "quit":
            self._stop_conversation_process()
            rospy.signal_shutdown("Quit via GUI")
            QApplication.quit()

    def _on_enter_state(self, new_state: GameState):
        self.current_state = new_state
        self._update_button_labels()
        self.lbl_status.setText(f"Status: {new_state.value}")

        if new_state == GameState.CALIBRATING:
            self.lbl_prompt.setText("Calibrating… please follow the on-screen hints.")
        rospy.loginfo(f"[GUI] → {new_state.value}")

        if new_state == GameState.IN_GAME:
            self.remaining_time = self.turn_timeout
            self.lbl_timer.setText(self._fmt_time(self.remaining_time))
            rospy.loginfo("[GUI] TIMER STARTED in InGame")
            self.ticker.start(1000)
        elif new_state == GameState.PAUSED:
            self.ticker.stop()
        else:
            self.ticker.stop()
            self.remaining_time = self.turn_timeout
            self.lbl_timer.setText("--:--")

        if new_state == GameState.INTRO:
            self.current_turn = 0
            self.current_score = 0
            self.lbl_turn.setText(f"Turn: 0/{self.total_rounds}")
            self.lbl_score.setText("Score: 0")
            self.lbl_prompt.setText("Welcome to Simon Says game with Flo robot!\n")
        elif new_state == GameState.IDLE:
            self.lbl_prompt.setText("Choose a setup step, or start the game directly.\n")
        elif new_state == GameState.READY:
            self.lbl_prompt.setText("Setup complete. Start the game, recalibrate, or replay instructions.\n")
        elif new_state == GameState.GAME_OVER:
            self.ticker.stop()
            self.remaining_time = self.turn_timeout
            self.lbl_timer.setText(self._fmt_time(self.remaining_time))

        self._update_buttons()

    def _tick_timer(self):
        if self.remaining_time > 0:
            self.remaining_time -= 1
            self.lbl_timer.setText(self._fmt_time(self.remaining_time))

    def _cb_score(self, msg: Int32):
        self.scoreUpdated.emit(msg.data)

    def _cb_prompt(self, msg: String):
        self.promptUpdated.emit(msg.data)

    def _cb_turn(self, msg: Int32):
        self.turnUpdated.emit(msg.data)

    def _cb_status(self, msg: String):
        self.statusUpdated.emit(msg.data)

    def _cb_ui_state(self, msg: String):
        self.uiStateUpdated.emit(msg.data)

    def _cb_user_text(self, msg: String):
        self.conversationTextReceived.emit("User", msg.data)

    def _cb_assistant_text(self, msg: String):
        self.conversationTextReceived.emit("Flo", msg.data)

    def _apply_score_update(self, score: int):
        self.current_score = score
        self.lbl_score.setText(f"Score: {self.current_score}")

    def _apply_prompt_update(self, text: str):
        self.lbl_prompt.setText(f"Prompt: {text}")

        if self.current_state == GameState.INTRO:
            self.rulesReceived.emit()

    def _apply_turn_update(self, turn_id: int):
        rospy.loginfo(f"[GUI] _cb_turn: ticker active? {self.ticker.isActive()}")
        self.current_turn = turn_id
        self.lbl_turn.setText(f"Turn: {self.current_turn}/{self.total_rounds}")
        if self.current_state == GameState.IN_GAME:
            self.remaining_time = self.turn_timeout
            self.lbl_timer.setText(self._fmt_time(self.remaining_time))

    def _apply_status_update(self, status: str):
        self.lbl_status.setText(f"Status: {status}")

    def _apply_ui_state_update(self, ui_state: str):
        self.backend_ui_state = ui_state
        self.ready_for_start = ui_state == "ready"
        if ui_state == "setup_calibration" and self.current_state != GameState.CALIBRATING:
            self.calibrationStarted.emit()
        elif ui_state == "ready" and self.current_state == GameState.CALIBRATING:
            self.calibrationFinished.emit()
        elif ui_state == "game_over":
            self.backendGameOver.emit()
        self._update_buttons()

    def _append_conversation_log(self, speaker: str, text: str):
        entry = text.strip()
        if not entry:
            return
        self.txt_conversation_log.appendPlainText(f"{speaker}: {entry}")
        scrollbar = self.txt_conversation_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _append_conversation_separator(self):
        document_text = self.txt_conversation_log.toPlainText().rstrip()
        if not document_text:
            return

        separator = "-" * 48
        if document_text.splitlines()[-1] == separator:
            return

        self.txt_conversation_log.appendPlainText(separator)
        scrollbar = self.txt_conversation_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _cb_preview_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logwarn(f"[GUI] Failed to decode preview image: {e}")
            return

        rgb = frame[:, :, ::-1]
        height, width, channels = rgb.shape
        bytes_per_line = channels * width
        image = QImage(
            rgb.tobytes(), width, height, bytes_per_line, QImage.Format_RGB888
        ).copy()
        self.cameraFrameReady.emit(image)

    def _update_camera_frame(self, image: QImage):
        self.current_camera_pixmap = QPixmap.fromImage(image)
        self._render_camera_pixmap()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._render_camera_pixmap()

    def _render_camera_pixmap(self):
        if self.current_camera_pixmap is None:
            return
        self.camera_view.set_frame(self.current_camera_pixmap)

    @staticmethod
    def _fmt_time(seconds: int) -> str:
        minutes, secs = divmod(seconds, 60)
        return f"{minutes:02d}:{secs:02d}"

    def _refresh_rounds_display(self):
        self.btn_rounds_value.setText(str(self.total_rounds))
        self.lbl_turn.setText(f"Turn: {self.current_turn}/{self.total_rounds}")

    def _set_total_rounds(self, rounds: int):
        rounds = max(self.min_rounds, int(rounds))
        if rounds == self.total_rounds:
            return

        self.total_rounds = rounds
        rospy.set_param("/game_runner/total_rounds", self.total_rounds)
        rospy.set_param("~total_rounds", self.total_rounds)
        rospy.loginfo(f"[GUI] Updated total_rounds to {self.total_rounds}")
        self._refresh_rounds_display()
        self._update_buttons()

    def _adjust_total_rounds(self, delta: int):
        if not self._rounds_edit_enabled():
            return
        self._set_total_rounds(self.total_rounds + delta)

    def _rounds_edit_enabled(self) -> bool:
        return self.current_state in (
            GameState.IDLE,
            GameState.INTRO_WAIT,
            GameState.READY,
            GameState.GAME_OVER,
        )

    def _update_button_labels(self):
        if self.current_state in (GameState.INTRO_WAIT, GameState.CALIBRATING):
            self.btn_stop.setText("Cancel Setup")
        elif self.current_state in (GameState.IN_GAME, GameState.PAUSED):
            self.btn_stop.setText("End Game")
        else:
            self.btn_stop.setText("Stop")

        if self.current_state == GameState.PAUSED:
            self.btn_pause_resume.setText("Resume Game")
        else:
            self.btn_pause_resume.setText("Pause Game")

        if self._is_conversation_running():
            self.btn_conversation.setText("Stop Conversation")
            self.btn_conversation.setProperty("running", "true")
        else:
            self.btn_conversation.setText("Start Conversation")
            self.btn_conversation.setProperty("running", "false")
        self.btn_conversation.style().unpolish(self.btn_conversation)
        self.btn_conversation.style().polish(self.btn_conversation)
        self.btn_conversation.update()

    def _update_buttons(self):
        for button in (
            self.btn_full_setup,
            self.btn_instructions,
            self.btn_calibrate,
            self.btn_start_game,
            self.btn_conversation,
            self.btn_pause_resume,
            self.btn_stop,
            self.btn_restart,
        ):
            button.setEnabled(False)

        state = self.current_state
        conversation_running = self._is_conversation_running()
        setup_allowed = not conversation_running
        game_running = state in (GameState.IN_GAME, GameState.PAUSED)

        if state == GameState.IDLE and self.ready_for_start:
            self.btn_full_setup.setEnabled(setup_allowed)
            self.btn_instructions.setEnabled(setup_allowed)
            self.btn_calibrate.setEnabled(setup_allowed)
            self.btn_start_game.setEnabled(not conversation_running)
        elif state == GameState.INTRO_WAIT:
            self.btn_stop.setEnabled(True)
            self.btn_restart.setEnabled(True)
            if self.ready_for_start:
                self.btn_full_setup.setEnabled(setup_allowed)
                self.btn_instructions.setEnabled(setup_allowed)
                self.btn_calibrate.setEnabled(setup_allowed)
                self.btn_start_game.setEnabled(not conversation_running)
        elif state == GameState.READY and self.ready_for_start:
            self.btn_full_setup.setEnabled(setup_allowed)
            self.btn_instructions.setEnabled(setup_allowed)
            self.btn_calibrate.setEnabled(setup_allowed)
            self.btn_start_game.setEnabled(not conversation_running)
            self.btn_restart.setEnabled(True)
        elif state == GameState.INTRO:
            self.btn_restart.setEnabled(True)
        elif state == GameState.CALIBRATING:
            self.btn_stop.setEnabled(True)
            self.btn_restart.setEnabled(True)
        elif state == GameState.IN_GAME:
            self.btn_pause_resume.setEnabled(True)
            self.btn_stop.setEnabled(True)
            self.btn_restart.setEnabled(True)
        elif state == GameState.PAUSED:
            self.btn_pause_resume.setEnabled(True)
            self.btn_stop.setEnabled(True)
            self.btn_restart.setEnabled(True)
        elif state == GameState.GAME_OVER:
            self.btn_restart.setEnabled(True)

        self.btn_conversation.setEnabled(not game_running)
        rounds_editable = self._rounds_edit_enabled()
        self.btn_rounds_down.setEnabled(rounds_editable and self.total_rounds > self.min_rounds)
        self.btn_rounds_up.setEnabled(rounds_editable)
        self._update_button_labels()

    def _is_conversation_running(self) -> bool:
        return self.conversation_process is not None and self.conversation_process.poll() is None

    def _toggle_conversation(self):
        if self._is_conversation_running():
            self._stop_conversation_process(add_log_separator=True)
        else:
            self._start_conversation_process()
        self._update_buttons()

    def _start_conversation_process(self):
        if self._is_conversation_running():
            return

        try:
            self.conversation_process = subprocess.Popen(
                ["rosrun", "flo_core", "conversation_agent.py"],
                start_new_session=True,
            )
            rospy.loginfo("[GUI] Started conversation agent")
        except Exception as exc:
            self.conversation_process = None
            rospy.logerr("[GUI] Failed to start conversation agent: %s", exc)

    def _stop_conversation_process(self, add_log_separator: bool = False):
        process = self.conversation_process
        self.conversation_process = None
        if process is None:
            return

        if process.poll() is None:
            try:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                rospy.logwarn("[GUI] Conversation agent did not stop after SIGINT; terminating")
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    rospy.logwarn("[GUI] Conversation agent did not terminate; killing")
                    process.kill()
                    process.wait(timeout=3)
            except Exception as exc:
                rospy.logwarn("[GUI] Failed to stop conversation agent cleanly: %s", exc)

        rospy.loginfo("[GUI] Stopped conversation agent")
        if add_log_separator:
            self._append_conversation_separator()

    def _sync_conversation_process(self):
        process = self.conversation_process
        if process is None or process.poll() is None:
            return

        exit_code = process.returncode
        self.conversation_process = None
        rospy.loginfo("[GUI] Conversation agent exited with code %s", exit_code)
        self._update_buttons()

    def closeEvent(self, event):
        self._stop_conversation_process()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    gui = SimonGUI()
    gui.showMaximized()
    sys.exit(app.exec_())


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
