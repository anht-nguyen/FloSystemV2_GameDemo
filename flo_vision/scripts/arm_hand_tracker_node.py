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
  ~PoseScore               (flo_msgs/PoseScore)

ROS Parameters (private, ~namespace)
------------------------------------
  preview            (bool, default: True)   – show OpenCV GUI window
  pose               (str,  default: "all")  – which gesture(s) to detect
  image              (str,  default: "/usb_cam/image_raw")

Author: 2025, FLO Robot project
"""

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String, Header
from flo_core_defs.msg import PoseScore
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import mediapipe as mp

# Helper with the geometric logic
from flo_vision.arm_tracker_helper import ArmTracker

# ──────────────────────────────────────────────────────
# MediaPipe drawing utilities
# ──────────────────────────────────────────────────────
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Force HighGUI to create its own GUI thread
cv2.startWindowThread()
cv2.namedWindow("FLO Vision – Arms & Hands", cv2.WINDOW_NORMAL)


class ArmHandTrackerNode:
    def __init__(self):
        # ---------- ROS housekeeping ----------
        self.bridge = CvBridge()
        self.preview = rospy.get_param("~preview", True)
        # self.pose_to_detect = rospy.get_param("~pose", "all")
        pose_param = rospy.get_param("~pose", "wave")
        if isinstance(pose_param,str):
            self.pose_to_detect = [p.strip() for p in pose_param.split(',') if p.strip()]
        else:
            self.pose_to_detect = ["wave"]

        # Topic names are param‑configurable so the same launch file can be reused
        image_topic = rospy.get_param("~image", "/usb_cam/image_raw")

        self.preview_frame = None  # shared buffer for the GUI loop

        # Publishers
        self.pub_left_elbow = rospy.Publisher("~left_elbow_angle", Float32, queue_size=10)
        self.pub_right_elbow = rospy.Publisher("~right_elbow_angle", Float32, queue_size=10)
        self.pub_left_shldr = rospy.Publisher("~left_shoulder_angle", Float32, queue_size=10)
        self.pub_right_shldr = rospy.Publisher("~right_shoulder_angle", Float32, queue_size=10)
        self.pub_gesture = rospy.Publisher("~gesture", String, queue_size=10)
        self.pub_posescore = rospy.Publisher("~pose_score", PoseScore, queue_size=10)
        # Subscriber
        rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber("~pose_command",String, self.pose_command_callback, queue_size= 1)
        # --------- MediaPipe initialisation ----------
        self.arm_tracker = ArmTracker()
        self.mp_pose = mp.solutions.pose
        self.mp_hands = mp.solutions.hands
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5,
                                      min_tracking_confidence=0.5)
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.5,
                                         min_tracking_confidence=0.5,
                                         max_num_hands=2)

        rospy.loginfo("arm_hand_tracker_node initialised – awaiting images…")
    def calculate_action_similarity(self, detected_gestures, required_gestures):
        if not required_gestures:
            return 0.0, False
           
    
        left_hand_actions = [g for g in required_gestures if 'left' in g or g in ['d_wave_left']]
        right_hand_actions = [g for g in required_gestures if 'right' in g or g in ['d_wave_right']]
        general_actions = [g for g in required_gestures if g not in left_hand_actions and g not in right_hand_actions]
       
        # if only one required gesture
        if len(required_gestures) == 1 and not left_hand_actions and not right_hand_actions:
            if required_gestures[0] in detected_gestures:
                return 1.0, True
            else:
                return 0.0, False
               
        # if two gestures
        if len(required_gestures) == 2 or (left_hand_actions and right_hand_actions):
            left_detected = any(action in detected_gestures for action in left_hand_actions)
            right_detected = any(action in detected_gestures for action in right_hand_actions)
            general_detected = any(action in detected_gestures for action in general_actions)
           
            total_parts = len(left_hand_actions) + len(right_hand_actions) + len(general_actions)
            detected_parts = 0
           
            if left_hand_actions and left_detected:
                detected_parts += 1
            if right_hand_actions and right_detected:
                detected_parts += 1
            if general_actions and general_detected:
                detected_parts += 1
               
            if detected_parts == total_parts:
                return 1.0, True
            elif detected_parts > 0:
                return 0.5, False
            else:
                return 0.0, False
               
        
        all_detected = all(action in detected_gestures for action in required_gestures)
        if all_detected:
            return 1.0, True
        elif any(action in detected_gestures for action in required_gestures):
            return 0.5, False
        else:
            return 0.0, False


    # ======================================================================
    #                            CALLBACK
    # ======================================================================
    def pose_command_callback(self, msg: String):
        """Examine the pose to detect"""
        new_pose = msg.data.strip().lower()
       
        pose_list = [p.strip() for p in new_pose.split(',') if p.strip()]
        valid_poses = ["all", "static", "wave", "swing_lateral", "reach_side", "raise", "d_wave_left", "d_wave_right","d_swing_lateral_left", "d_swing_lateral_right", "s_raise_left", "s_raise_right","s_reach_side_left", "s_reach_side_right"]
        invalid_poses = [p for p in pose_list if p not in valid_poses]
       
        if invalid_poses:
            rospy.logwarn(f"Invalid pose type(s): {invalid_poses}. Valid types: {valid_poses}")
            return
       
        if pose_list:
            old_pose = self.pose_to_detect
            self.pose_to_detect = pose_list 
            rospy.loginfo(f"Pose detection changed from '{old_pose}' to '{pose_list}'")
        else:
            rospy.logwarn("Empty pose list received")

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
        
        frame = cv2.flip(frame, 1)

        # ----- MediaPipe inference -----
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        pose_results = self.pose.process(rgb)
        hand_results = self.hands.process(rgb)
        rgb.flags.writeable = True
        image_bgr = frame  # keep original for drawing

        h, w = image_bgr.shape[:2]

        # ------------------------------------------------------------------
        #               DRAW POSE & HAND LANDMARKS (NEW)
        # ------------------------------------------------------------------
        if pose_results.pose_landmarks:
            mp_drawing.draw_landmarks(
                image_bgr,
                pose_results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style(),
            )

        if hand_results.multi_hand_landmarks:
            for hlm in hand_results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    image_bgr,
                    hlm,
                    self.mp_hands.HAND_CONNECTIONS,
                )

        # ------------------------------------------------------------------
        #                  ARM / SHOULDER ANGLES + GESTURES
        # ------------------------------------------------------------------
        if pose_results.pose_landmarks:
            landmarks = pose_results.pose_landmarks.landmark

            l_sh, l_el, l_wr, l_el_ang, l_sh_ang = self.arm_tracker.get_arm_info(landmarks, side="left")
            r_sh, r_el, r_wr, r_el_ang, r_sh_ang = self.arm_tracker.get_arm_info(landmarks, side="right")

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
            detected_gestures = []
            # examine if the list contain waving
            if "wave" in self.pose_to_detect or "all" in self.pose_to_detect:
                if self.arm_tracker.is_arm_wave(landmarks, image_bgr):
                    detected_gestures.append("wave")
           
            # examine if the list contain swing_lateral
            if "swing_lateral" in self.pose_to_detect or "all" in self.pose_to_detect:
                if self.arm_tracker.is_arm_swing_lateral(landmarks, image_bgr):
                    detected_gestures.append("swing_lateral")
           
            # examine if the list contain swing_forward
            if "swing_forward" in self.pose_to_detect or "all" in self.pose_to_detect:
                if self.arm_tracker.is_arm_swing_forward(landmarks, image_bgr):
                    detected_gestures.append("swing_forward")
           
            # examine if the list contain raise
            if "raise" in self.pose_to_detect or "all" in self.pose_to_detect:
                if self.arm_tracker.is_arm_raise(landmarks, image_bgr):
                    detected_gestures.append("raise")
            if "reach_side" in self.pose_to_detect or "all" in self.pose_to_detect:
                if self.arm_tracker.is_arm_reach_side(landmarks, image_bgr):
                    detected_gestures.append("reach_side")

            if "d_wave_left" in self.pose_to_detect or "all" in self.pose_to_detect:
                if self.arm_tracker.is_single_arm_wave(landmarks, image_bgr, side='left'):
                    detected_gestures.append("d_wave_left")
            
            if "d_wave_right" in self.pose_to_detect or "all" in self.pose_to_detect:
                if self.arm_tracker.is_single_arm_wave(landmarks, image_bgr, side='right'):
                    detected_gestures.append("d_wave_right")

            if "d_swing_lateral_left" in self.pose_to_detect or "all" in self.pose_to_detect:
                if self.arm_tracker.is_single_arm_swing_lateral(landmarks, image_bgr, side='left'):
                    detected_gestures.append("d_swing_lateral_left")
            if "d_swing_lateral_right" in self.pose_to_detect or "all" in self.pose_to_detect:
                if self.arm_tracker.is_single_arm_swing_lateral(landmarks, image_bgr, side='right'):
                    detected_gestures.append("d_swing_lateral_right")

            if "s_raise_left" in self.pose_to_detect or "all" in self.pose_to_detect:
                if self.arm_tracker.is_single_arm_raise(landmarks, image_bgr, side='left'):
                    detected_gestures.append("s_raise_left")

            if "s_raise_right" in self.pose_to_detect or "all" in self.pose_to_detect:
                if self.arm_tracker.is_single_arm_raise(landmarks, image_bgr, side='right'):
                    detected_gestures.append("s_raise_right")
            
            if "s_reach_side_left" in self.pose_to_detect or "all" in self.pose_to_detect:
                if self.arm_tracker.is_single_arm_reach_side(landmarks, image_bgr, side='left'):
                    detected_gestures.append("s_reach_side_left")

            if "s_reach_side_right" in self.pose_to_detect or "all" in self.pose_to_detect:
                if self.arm_tracker.is_single_arm_reach_side(landmarks, image_bgr, side='right'):
                    detected_gestures.append("s_reach_side_right")

            if "static" in self.pose_to_detect or "all" in self.pose_to_detect:
                if self.arm_tracker.is_static_pose(landmarks, image_bgr):
                    detected_gestures.append("static")

            # publish the gesture
            if detected_gestures:
                gesture_txt = ",".join(detected_gestures)
                self.pub_gesture.publish(gesture_txt)
            
            similarity, matched = self.calculate_action_similarity(detected_gestures, self.pose_to_detect)

            pose_score = PoseScore()
            pose_score.header = Header()
            pose_score.header.stamp = msg.header.stamp if hasattr(msg,  'header') else rospy.Time.now()
            pose_score.header.frame_id = "camera_frame"
            pose_score.matched = matched
            pose_score.similarity = similarity
            
            self.pub_posescore.publish(pose_score)
            # Optionally overlay angle values on‑screen
            if self.preview and l_sh is not None and r_sh is not None:
                cv2.putText(image_bgr, f"{int(l_el_ang)}", (int(l_el[0] * w), int(l_el[1] * h)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(image_bgr, f"{int(r_el_ang)}", (int(r_el[0] * w), int(r_el[1] * h)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
        # ------------------------------------------------------------------
        #                  OPTIONAL GUI WINDOW
        # ------------------------------------------------------------------
        if self.preview:
            self.preview_frame = image_bgr  # stash latest frame for main‑thread GUI


# ======================================================================
#                           MAIN PROGRAM LOOP
# ======================================================================

def main():
    rospy.init_node("arm_hand_tracker")
    tracker = ArmHandTrackerNode()
    rate = rospy.Rate(30)  # ≈30 Hz GUI refresh

    try:
        while not rospy.is_shutdown():
            if tracker.preview and tracker.preview_frame is not None:
                cv2.imshow("FLO Vision – Arms & Hands", tracker.preview_frame)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC quits preview
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
