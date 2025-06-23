#!/usr/bin/env python3
"""
arm_hand_tracker_node.py  – FLO‑Vision ROS node

Subscribes
----------
  ~image        (sensor_msgs/Image)   – RGB frames from a camera driver
    • default: /camera/color/image_raw

Publishes
---------
  ~left_elbow_angle        (std_msgs/Float32)
  ~right_elbow_angle       (std_msgs/Float32)
  ~left_shoulder_angle     (std_msgs/Float32)
  ~right_shoulder_angle    (std_msgs/Float32)
  ~gesture                 (std_msgs/String)    – detected high‑level action

ROS Parameters (private, ~namespace)
------------------------------------
  preview            (bool, default: True)   – show OpenCV GUI window
  pose               (str,  default: "all")  – which gesture(s) to detect
  image              (str,  default: "/camera/color/image_raw")

Author: 2025, FLO Robot project
"""

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import mediapipe as mp

# Helper with all geometric logic ported from the original scripts
from flo_vision.arm_tracker_helper import ArmTracker   

# ----- OpenCV window initialisation (must run in *main* thread) -----
cv2.namedWindow("FLO Vision – Arms & Hands", cv2.WINDOW_NORMAL)

class ArmHandTrackerNode:
    def __init__(self):
        # ---------- ROS housekeeping ----------
        self.bridge = CvBridge()
        self.preview = rospy.get_param("~preview", True)
        self.pose_to_detect = rospy.get_param("~pose", "all")

        image_topic = rospy.get_param("~image", "/usb_cam/image_raw")

        # Frame buffer shared with the main‑loop GUI refresher
        self.preview_frame = None

        # Publishers
        self.pub_left_elbow   = rospy.Publisher("~left_elbow_angle",   Float32, queue_size=10)
        self.pub_right_elbow  = rospy.Publisher("~right_elbow_angle",  Float32, queue_size=10)
        self.pub_left_shldr   = rospy.Publisher("~left_shoulder_angle",Float32, queue_size=10)
        self.pub_right_shldr  = rospy.Publisher("~right_shoulder_angle",Float32, queue_size=10)
        self.pub_gesture      = rospy.Publisher("~gesture",            String,  queue_size=10)

        # Subscriber – image acquisition happens in its own thread (rospy’s design)
        rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)

        # --------- MediaPipe initialisation ----------
        self.arm_tracker = ArmTracker()                              # domain logic
        self.mp_pose = mp.solutions.pose
        self.mp_hands = mp.solutions.hands
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5,
                                      min_tracking_confidence=0.5)
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.5,
                                         min_tracking_confidence=0.5,
                                         max_num_hands=2)

        rospy.loginfo("arm_hand_tracker_node initialised – awaiting images…")

    # ==================================================================
    #                            CALLBACK
    # ==================================================================
    def image_callback(self, msg: Image):
        """Main image processing pipeline."""
        try:
            encoding = "bgr8" if msg.encoding.endswith("bgr8") else "rgb8"
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
            if encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except CvBridgeError as e:
            rospy.logwarn(f"CvBridge conversion failed: {e}")
            return

        if not hasattr(self, "_notified"):
            rospy.loginfo(f"[Tracker] first frame sum = {frame.sum()}")
            self._notified = True

        # ----- MediaPipe inference -----
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        pose_results  = self.pose.process(rgb)
        hand_results  = self.hands.process(rgb)
        rgb.flags.writeable = True
        image_bgr = frame  # keep original for drawing

        # Dimensions for text overlay
        h, w = image_bgr.shape[:2]

        # ------------------------------------------------------------------
        #                  ARM / SHOULDER ANGLES + GESTURES
        # ------------------------------------------------------------------
        if pose_results.pose_landmarks:
            landmarks = pose_results.pose_landmarks.landmark

            # get_arm_info() returns (shoulder, elbow, wrist, elbow_angle, shoulder_angle)
            l_sh, l_el, l_wr, l_el_ang, l_sh_ang = self.arm_tracker.get_arm_info(
                landmarks, side="left")
            r_sh, r_el, r_wr, r_el_ang, r_sh_ang = self.arm_tracker.get_arm_info(
                landmarks, side="right")

            # Publish angles when valid
            if l_el_ang is not None:
                self.pub_left_elbow.publish(l_el_ang)
            if r_el_ang is not None:
                self.pub_right_elbow.publish(r_el_ang)
            if l_sh_ang is not None:
                self.pub_left_shldr.publish(l_sh_ang)
            if r_sh_ang is not None:
                self.pub_right_shldr.publish(r_sh_ang)

            # ---------------- Gesture logic ----------------
            gesture_txt = ""
            if self.pose_to_detect in ("wave", "all"):
                self.arm_tracker.is_arm_wave(landmarks, image_bgr, self.pose_to_detect)
                gesture_txt = "wave"
            if self.pose_to_detect in ("swing_lateral", "all"):
                self.arm_tracker.is_arm_swing_lateral(landmarks, image_bgr)
                gesture_txt = "swing_lateral"
            if self.pose_to_detect in ("swing_forward", "all"):
                self.arm_tracker.is_arm_swing_forward(landmarks, image_bgr)
                gesture_txt = "swing_forward"
            if self.pose_to_detect in ("raise", "all"):
                self.arm_tracker.is_arm_raise(landmarks, image_bgr)
                gesture_txt = "raise"

            if gesture_txt:
                self.pub_gesture.publish(gesture_txt)

            # Optionally overlay angle values on‑screen
            if self.preview and l_sh is not None and r_sh is not None:
                cv2.putText(image_bgr, f"{int(l_el_ang)}",
                            (int(l_el[0]*w), int(l_el[1]*h)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(image_bgr, f"{int(r_el_ang)}",
                            (int(r_el[0]*w), int(r_el[1]*h)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # ------------------------------------------------------------------
        #                  PASS FRAME TO GUI REFRESHER
        # ------------------------------------------------------------------
        if self.preview:
            self.preview_frame = image_bgr


def main():
    rospy.init_node("arm_hand_tracker")
    tracker = ArmHandTrackerNode()
    rate = rospy.Rate(30)  # ~30 Hz GUI refresh

    try:
        while not rospy.is_shutdown():
            if tracker.preview and tracker.preview_frame is not None:
                cv2.imshow("FLO Vision – Arms & Hands", tracker.preview_frame)
                # ESC key closes the preview window only (node keeps running)
                if cv2.waitKey(1) & 0xFF == 27:
                    rospy.loginfo("ESC pressed, quitting preview.")
                    tracker.preview = False
                    cv2.destroyAllWindows()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
