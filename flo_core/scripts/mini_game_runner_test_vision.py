import rospy
from flo_core_defs.msg import PoseScore

class MiniGameRunner:
    def __init__(self):
        # Read duration and threshold from ROS parameters
        self.duration = rospy.get_param('~duration', 5.0)
        self.threshold = rospy.get_param('~threshold', 0.5)

        rospy.loginfo(f"[MiniGameRunner] Starting: duration={self.duration}s, threshold={self.threshold*100:.0f}%")

        # Counters
        self.total = 0
        self.matched = 0

        # Subscriber
        self.sub = rospy.Subscriber(
            '/arm_hand_tracker/pose_score', PoseScore, self.callback)

        # Timer to end test after duration
        rospy.Timer(rospy.Duration(self.duration), self.finish, oneshot=True)

    def callback(self, msg):
        # Log each incoming message
        self.total += 1
        if msg.matched:
            self.matched += 1
        rospy.loginfo(f"[MiniGameRunner] Received PoseScore: matched={msg.matched}, similarity={msg.similarity:.2f}")

    def finish(self, event):
        # Compute ratio and decide
        if self.total == 0:
            ratio = 0.0
        else:
            ratio = float(self.matched) / self.total
        result = 'SUCCESS' if ratio > self.threshold else 'FAILURE'
        rospy.loginfo("[MiniGameRunner] Test complete: total_msgs=%d, matched_msgs=%d, ratio=%.2f -> %s", 
                      self.total, self.matched, ratio, result)
        rospy.signal_shutdown("Test finished")

if __name__ == '__main__':
    rospy.init_node('mini_game_runner')
    runner = MiniGameRunner()
    rospy.spin()
