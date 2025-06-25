#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Main controller node for humanoid robot arm actions.
Imports and uses ActionSequenceController to execute predefined action sequences.
Removes any gripper, AprilTag, and MQTT dependencies.
Listens for action commands as Strings in format '<ACTION_NAME>_<SIDE>'.
E.g., 'D_WAVE_right' or 'S_RAISE_left'.
"""
import sys
import rospy
import moveit_commander
from std_msgs.msg import String

from flo_core.action_sequence_controller import ActionSequenceController, Action


def main():
    # Initialize MoveIt and ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('action_sequence_controller_node')

    # Reference frame for all planning
    reference_frame = 'world'

    # Initialize MoveIt groups for right, left arms
    arm_R = moveit_commander.MoveGroupCommander('R')
    arm_L = moveit_commander.MoveGroupCommander('L')

    # Common planning settings
    for grp in (arm_R, arm_L):
        grp.set_pose_reference_frame(reference_frame)
        grp.allow_replanning(True)
        grp.set_goal_position_tolerance(0.001)
        grp.set_goal_orientation_tolerance(1)
        grp.set_max_acceleration_scaling_factor(0.6)
        grp.set_max_velocity_scaling_factor(0.5)


    # Instantiate the action sequence controller
    controller = ActionSequenceController(
        arm_R=arm_R,
        arm_L=arm_L,
    )

    def command_callback(msg: String):
        try:
            tokens = msg.data.split('_')
            action_name = '_'.join(tokens[:-1])
            side = tokens[-1]
            action = Action[action_name]
            rospy.loginfo(f"Received command: {action.name} on {side}-arm")
            controller.execute_action(action, side, reference_frame)
        except KeyError:
            rospy.logerr(f"Invalid action name: {msg.data}")
        except Exception as e:
            rospy.logerr(f"Error parsing command '{msg.data}': {e}")

    # Subscribe to action command topic
    rospy.Subscriber('/action_command', String, command_callback)

    rospy.loginfo("Action sequence controller node is up and running.")
    rospy.spin()

    # Shutdown MoveIt cleanly on exit
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
