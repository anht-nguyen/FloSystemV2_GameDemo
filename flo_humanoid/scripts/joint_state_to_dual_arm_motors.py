#!/usr/bin/env python3
"""
Bridge MoveIt! joint trajectories (/joint_states) to Dynamixel motor commands via flo_humanoid's SetArmsJointPositions message, using YAML-defined IDs and offsets.
"""
import rospy
from sensor_msgs.msg import JointState
from flo_humanoid.msg import SetArmsJointPositions
import math

NODE_NAME = 'joint_state_to_dual_arm_motors'
TOPIC_JOINT_STATES = '/joint_states'
TOPIC_SET_POSITIONS = '/set_arms_joint_positions'

class JointStateToDxlBridge:
    def __init__(self):
        rospy.init_node(NODE_NAME)
        # Load joint ID map and mechanical offsets (in degrees) from ROS params
        self.joint_id_map = rospy.get_param('joint_id_map')
        self.offsets = rospy.get_param('offsets')

        self.pub = rospy.Publisher(TOPIC_SET_POSITIONS, SetArmsJointPositions, queue_size=1)
        rospy.Subscriber(TOPIC_JOINT_STATES, JointState, self.joint_states_callback)

        rospy.loginfo(f"[{NODE_NAME}] Bridging {TOPIC_JOINT_STATES} → {TOPIC_SET_POSITIONS}")

    def convert_to_dynamixel_position(self, angle_deg):
        """
        Convert a joint angle in degrees (0-360) to a 12-bit Dynamixel position (0-4095).
        """
        angle = angle_deg % 360.0
        return int((angle / 360.0) * 4096.0)

    def joint_states_callback(self, msg):
        js = dict(zip(msg.name, msg.position))

        try:
            r1 = math.degrees(js['r1']) + self.offsets['r1']
            r2 = math.degrees(js['r2']) + self.offsets['r2']
            r3 = math.degrees(js['r3']) + self.offsets['r3']
            r4 = - math.degrees(js['r4']) + self.offsets['r4']

            l1 = math.degrees(js['l1']) + self.offsets['l1']
            l2 = math.degrees(js['l2']) + self.offsets['l2']
            l3 = math.degrees(js['l3']) + self.offsets['l3']
            l4 = math.degrees(js['l4']) + self.offsets['l4'] 
        except KeyError as e:
            rospy.logwarn(f"Joint '{e.args[0]}' not found in /joint_states, skipping.")
            return

        cmd = SetArmsJointPositions(
            # IDs from YAML
            self.joint_id_map['l1'], self.joint_id_map['l2'], self.joint_id_map['l3'], self.joint_id_map['l4'],
            self.joint_id_map['r1'], self.joint_id_map['r2'], self.joint_id_map['r3'], self.joint_id_map['r4'],
            # Command type for each: all 'position'
            'position', 'position', 'position', 'position',
            'position', 'position', 'position', 'position',
            # Values: convert angles to DXL ticks
            self.convert_to_dynamixel_position(l1),
            self.convert_to_dynamixel_position(l2),
            self.convert_to_dynamixel_position(l3),
            self.convert_to_dynamixel_position(l4),
            self.convert_to_dynamixel_position(r1),
            self.convert_to_dynamixel_position(r2),
            self.convert_to_dynamixel_position(r3),
            self.convert_to_dynamixel_position(r4)
        )

        # cmd = SetArmsJointPositions(
        #     # IDs from YAML
        #     self.joint_id_map['l1'], self.joint_id_map['l2'], self.joint_id_map['l3'], self.joint_id_map['l4'],
        #     0,0,0,0,
        #     # Command type for each: all 'position'
        #     'position', 'position', 'position', 'position',
        #     '', '', '', '',
        #     # Values: convert angles to DXL ticks
        #     self.convert_to_dynamixel_position(l1),
        #     self.convert_to_dynamixel_position(l2),
        #     self.convert_to_dynamixel_position(l3),
        #     self.convert_to_dynamixel_position(l4),
        #     0,
        #     0,
        #     0,
        #     0
        # )

        
        self.pub.publish(cmd)
        rospy.logdebug(
            f"Published DXL positions: L[{l1:.1f},{l2:.1f},{l3:.1f},{l4:.1f}], "
            f"R[{r1:.1f},{r2:.1f},{r3:.1f},{r4:.1f}]"
        )


if __name__ == '__main__':
    try:
        bridge = JointStateToDxlBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
