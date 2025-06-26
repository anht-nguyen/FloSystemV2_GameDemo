#!/usr/bin/env python3
"""
A simple ROS-integrated GUI for controlling the Simon Says game.
Provides buttons for Start, Pause, Stop, Quit, a real-time Scoreboard, a Round Timer countdown, and displays prompts and turn counter.
"""
import sys
import rospy
from std_msgs.msg import String, Int32
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QHBoxLayout, QFrame
from PyQt5.QtCore import Qt, QTimer

class SimonGUI(QWidget):
    def __init__(self):
        super(SimonGUI, self).__init__()
        # Initialize ROS node and communication
        rospy.init_node('simon_gui', anonymous=True)
        self.pub = rospy.Publisher('/simon_game/control', String, queue_size=10)
        rospy.Subscriber('/simon_game/score', Int32, self.update_score)
        rospy.Subscriber('/simon_game/prompt', String, self.update_prompt)
        rospy.Subscriber('/simon_game/turn_id', Int32, self.update_turn)

        # Parameters
        self.turn_timeout = rospy.get_param('~turn_timeout', 4)  # seconds
        self.total_rounds = rospy.get_param('~total_rounds', 15)
        self.remaining_time = self.turn_timeout

        # Build UI and timer
        self.init_ui()
        self.init_timer()

    def init_ui(self):
        self.setWindowTitle('Simon Says Control Panel')
        self.setFixedSize(350, 380)

        main_layout = QVBoxLayout()
        main_layout.setAlignment(Qt.AlignTop)

        # Prompt display
        self.prompt_label = QLabel('Prompt: --')
        self.prompt_label.setWordWrap(True)
        self.prompt_label.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        main_layout.addWidget(self.prompt_label)

        # Turn counter display
        self.turn_label = QLabel(f'Turn: 0/{self.total_rounds}')
        self.turn_label.setAlignment(Qt.AlignCenter)
        self.turn_label.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        main_layout.addWidget(self.turn_label)

        # Status display
        self.status_label = QLabel('Status: Idle')
        self.status_label.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        main_layout.addWidget(self.status_label)

        # Buttons layout
        btn_layout = QHBoxLayout()
        for text, cmd in [('Start', 'start'), ('Pause', 'pause'), ('Stop', 'stop'), ('Quit', 'quit')]:
            btn = QPushButton(text)
            btn.clicked.connect(lambda _, c=cmd: self.send_command(c))
            btn_layout.addWidget(btn)
        main_layout.addLayout(btn_layout)

        # Scoreboard
        self.score_label = QLabel('Score: 0')
        self.score_label.setAlignment(Qt.AlignCenter)
        self.score_label.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        main_layout.addWidget(self.score_label)

        # Round Timer display
        self.timer_label = QLabel(f'Time: {self.format_time(self.remaining_time)}')
        self.timer_label.setAlignment(Qt.AlignCenter)
        self.timer_label.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        main_layout.addWidget(self.timer_label)

        self.setLayout(main_layout)

    def init_timer(self):
        self.gui_timer = QTimer(self)
        self.gui_timer.timeout.connect(self.tick)

    def send_command(self, cmd):
        self.pub.publish(cmd)
        self.status_label.setText(f'Status: {cmd.capitalize()}')

        if cmd == 'start':
            # reset timer
            self.remaining_time = self.turn_timeout
            self.timer_label.setText(f'Time: {self.format_time(self.remaining_time)}')
            self.turn_label.setText(f'Turn: 0/{self.total_rounds}')
            self.gui_timer.start(1000)
        elif cmd == 'pause':
            self.gui_timer.stop()
        elif cmd == 'stop':
            self.gui_timer.stop()
            self.remaining_time = self.turn_timeout
            self.timer_label.setText(f'Time: {self.format_time(self.remaining_time)}')
            self.turn_label.setText(f'Turn: 0/{self.total_rounds}')
        elif cmd == 'quit':
            rospy.signal_shutdown('Quit via GUI')
            QApplication.quit()

    def tick(self):
        if self.remaining_time > 0:
            self.remaining_time -= 1
            self.timer_label.setText(f'Time: {self.format_time(self.remaining_time)}')
        else:
            self.gui_timer.stop()
            self.status_label.setText('Status: Time Up')
            self.pub.publish('round_end')

    def update_score(self, msg):
        self.score_label.setText(f'Score: {msg.data}')

    def update_prompt(self, msg):
        self.prompt_label.setText(f'Prompt: {msg.data}')

    def update_turn(self, msg):
        # Display current turn over total rounds
        self.turn_label.setText(f'Turn: {msg.data}/{self.total_rounds}')

    @staticmethod
    def format_time(seconds):
        mins, secs = divmod(seconds, 60)
        return f"{mins:02d}:{secs:02d}"

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = SimonGUI()
    gui.show()
    sys.exit(app.exec_())
