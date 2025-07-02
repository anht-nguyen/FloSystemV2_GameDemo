#!/usr/bin/env python3
"""
Bridge MoveIt! joint trajectories (/joint_states) to Dynamixel motor commands via flo_humanoid's SetArmsJointPositions message.
"""
import rospy
from sensor_msgs.msg import JointState
from flo_humanoid.msg import SetArmsJointPositions
import math

NODE_NAME = 'joint_state_to_dual_arm_motors'
TOPIC_JOINT_STATES = '/joint_states'
TOPIC_SET_POSITIONS = '/set_arms_joint_positions'


def convert_to_dynamixel_position(angle_deg):
    """
    Convert a joint angle in degrees (0-360) to a 12-bit Dynamixel position (0-4095).
    """
    angle = angle_deg % 360.0
    return int((angle / 360.0) * 4096.0)


def joint_states_callback(msg, publisher):
    """
    Called on each MoveIt! joint state message; converts to DXL positions and publishes.
    """
    js = dict(zip(msg.name, msg.position))

    try:
        r1 = math.degrees(js['r1']) + 110
        r2 = math.degrees(js['r2']) + 95
        r3 = math.degrees(js['r3']) + 180
        r4 = math.degrees(js['r4']) + 87

        l1 = math.degrees(js['l1']) + 300
        l2 = math.degrees(js['l2']) + 270
        l3 = math.degrees(js['l3']) + 180
        l4 = 180 - math.degrees(js['l4'])
    except KeyError as e:
        rospy.logwarn(f"Joint '{e.args[0]}' not found in /joint_states, skipping.")
        return

    cmd = SetArmsJointPositions(
        # Motor IDs: left arm (111-122), right arm (211-222)
        111, 112, 121, 122,
        211, 212, 221, 222,
        # Command type for each: all 'position'
        'position', 'position', 'position', 'position',
        'position', 'position', 'position', 'position',
        # Values: l1, l2, l3, l4, r1, r2, r3, r4
        convert_to_dynamixel_position(l1),
        convert_to_dynamixel_position(l2),
        convert_to_dynamixel_position(l3),
        convert_to_dynamixel_position(l4),
        convert_to_dynamixel_position(r1),
        convert_to_dynamixel_position(r2),
        convert_to_dynamixel_position(r3),
        convert_to_dynamixel_position(r4)
    )

    publisher.publish(cmd)
    rospy.logdebug(
        f"Published DXL positions: L[{l1:.1f},{l2:.1f},{l3:.1f},{l4:.1f}], "
        f"R[{r1:.1f},{r2:.1f},{r3:.1f},{r4:.1f}]"
    )


def main():
    rospy.init_node(NODE_NAME)
    pub = rospy.Publisher(TOPIC_SET_POSITIONS, SetArmsJointPositions, queue_size=1)

    rospy.Subscriber(TOPIC_JOINT_STATES, JointState, joint_states_callback, callback_args=pub)

    rospy.loginfo(f"[{NODE_NAME}] Bridging {TOPIC_JOINT_STATES} → {TOPIC_SET_POSITIONS}")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
