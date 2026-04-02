#!/usr/bin/env python3
"""
ROS GUI for Dual‑Arm Simon Says – merged camera + controller window.
"""
from __future__ import annotations

import sys
from enum import Enum

import rospy
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtCore import Qt, QState, QStateMachine, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPainter, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
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


class SimonGUI(QWidget):
    """Main GUI widget driven by a Qt StateMachine."""

    fullSetupClicked = pyqtSignal()
    instructionsClicked = pyqtSignal()
    calibrateClicked = pyqtSignal()
    startGameClicked = pyqtSignal()
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
    backendGameOver = pyqtSignal()

    def __init__(self):
        super().__init__()

        rospy.init_node("simon_gui", anonymous=True)
        self.cmd_pub = rospy.Publisher("/simon_game/control", String, queue_size=10)
        self.turn_timeout = int(rospy.get_param("/game_runner/turn_timeout"))
        self.total_rounds = int(rospy.get_param("/game_runner/total_rounds"))

        self.current_turn = 0
        self.current_score = 0
        self.remaining_time = self.turn_timeout
        self.current_state: GameState = GameState.IDLE
        self.ready_for_start = False
        self.backend_ui_state = "booting"
        self.bridge = CvBridge()
        self.current_camera_pixmap = None

        self._build_ui()
        self.cameraFrameReady.connect(self._update_camera_frame)
        self.scoreUpdated.connect(self._apply_score_update)
        self.promptUpdated.connect(self._apply_prompt_update)
        self.turnUpdated.connect(self._apply_turn_update)
        self.statusUpdated.connect(self._apply_status_update)
        self.uiStateUpdated.connect(self._apply_ui_state_update)

        self.ticker = QTimer(self)
        self.ticker.timeout.connect(self._tick_timer)

        self._build_fsm()
        self._update_buttons()

        # Subscribe after UI/state setup so early messages cannot hit half-built objects.
        rospy.Subscriber("/simon_game/score", Int32, self._cb_score)
        rospy.Subscriber("/simon_game/prompt", String, self._cb_prompt)
        rospy.Subscriber("/simon_game/turn_id", Int32, self._cb_turn)
        rospy.Subscriber("/simon_game/status", String, self._cb_status)
        rospy.Subscriber("/simon_game/ui_state", String, self._cb_ui_state)
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
            QLabel#promptBox {
                background: #1a2735;
                border: 1px solid #2e4359;
                border-radius: 12px;
                padding: 14px;
                font-size: 18px;
            }
            QPushButton {
                background: #2b8a78;
                border: none;
                border-radius: 12px;
                color: white;
                font-size: 17px;
                font-weight: 700;
                min-height: 52px;
                padding: 10px 14px;
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
        control_layout.setAlignment(Qt.AlignTop)

        title = QLabel("Game Controller")
        title.setStyleSheet("font-size: 24px; font-weight: 700;")
        control_layout.addWidget(title)

        self.lbl_prompt = QLabel("Prompt: –")
        self.lbl_prompt.setObjectName("promptBox")
        self.lbl_prompt.setWordWrap(True)
        control_layout.addWidget(self.lbl_prompt)

        self.lbl_turn = QLabel(f"Turn: 0/{self.total_rounds}")
        self.lbl_score = QLabel("Score: 0")
        for widget in (self.lbl_turn, self.lbl_score):
            widget.setAlignment(Qt.AlignCenter)
            widget.setObjectName("metric")
        row = QHBoxLayout()
        row.addWidget(self.lbl_turn)
        row.addWidget(self.lbl_score)
        control_layout.addLayout(row)

        self.lbl_timer = QLabel(self._fmt_time(self.remaining_time))
        self.lbl_timer.setAlignment(Qt.AlignCenter)
        self.lbl_timer.setObjectName("metric")
        control_layout.addWidget(self.lbl_timer)

        self.lbl_status = QLabel(f"Status: {self.current_state.value}")
        self.lbl_status.setObjectName("metric")
        control_layout.addWidget(self.lbl_status)

        self.btn_full_setup = QPushButton("Run Full Setup")
        self.btn_instructions = QPushButton("Read Instructions")
        self.btn_calibrate = QPushButton("Calibrate Camera")
        self.btn_start_game = QPushButton("Start Game")
        self.btn_pause_resume = QPushButton("Pause Game")
        self.btn_stop = QPushButton("Stop")
        self.btn_restart = QPushButton("Restart Session")
        self.btn_quit = QPushButton("Quit")
        self.btn_stop.setObjectName("danger")
        self.btn_quit.setObjectName("secondary")

        self._cmd_map = {
            self.btn_full_setup: ("run_full_setup", self.fullSetupClicked),
            self.btn_instructions: ("read_instructions", self.instructionsClicked),
            self.btn_calibrate: ("calibrate_camera", self.calibrateClicked),
            self.btn_start_game: ("start_game", self.startGameClicked),
            self.btn_pause_resume: ("pause_resume", self.pauseResumeClicked),
            self.btn_stop: ("stop", self.stopClicked),
            self.btn_restart: ("restart", self.restartClicked),
            self.btn_quit: ("quit", self.quitClicked),
        }

        row1 = QHBoxLayout()
        for button in (self.btn_full_setup, self.btn_instructions):
            button.clicked.connect(self._on_button)
            row1.addWidget(button)
        control_layout.addLayout(row1)

        row2 = QHBoxLayout()
        for button in (self.btn_calibrate, self.btn_start_game, self.btn_pause_resume):
            button.clicked.connect(self._on_button)
            row2.addWidget(button)
        control_layout.addLayout(row2)

        row3 = QHBoxLayout()
        for button in (self.btn_stop, self.btn_restart, self.btn_quit):
            button.clicked.connect(self._on_button)
            row3.addWidget(button)
        control_layout.addLayout(row3)
        control_layout.addStretch(1)

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

    def _update_buttons(self):
        for button in (
            self.btn_full_setup,
            self.btn_instructions,
            self.btn_calibrate,
            self.btn_start_game,
            self.btn_pause_resume,
            self.btn_stop,
            self.btn_restart,
            self.btn_quit,
        ):
            button.setEnabled(False)

        state = self.current_state
        if state == GameState.IDLE and self.ready_for_start:
            self.btn_full_setup.setEnabled(True)
            self.btn_instructions.setEnabled(True)
            self.btn_calibrate.setEnabled(True)
            self.btn_start_game.setEnabled(True)
        elif state == GameState.INTRO_WAIT:
            self.btn_stop.setEnabled(True)
            self.btn_restart.setEnabled(True)
            if self.ready_for_start:
                self.btn_full_setup.setEnabled(True)
                self.btn_instructions.setEnabled(True)
                self.btn_calibrate.setEnabled(True)
                self.btn_start_game.setEnabled(True)
        elif state == GameState.READY and self.ready_for_start:
            self.btn_full_setup.setEnabled(True)
            self.btn_instructions.setEnabled(True)
            self.btn_calibrate.setEnabled(True)
            self.btn_start_game.setEnabled(True)
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

        self.btn_quit.setEnabled(True)


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
