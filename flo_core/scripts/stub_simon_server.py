#!/usr/bin/env python3
import rospy, time
import actionlib
from flo_core_defs.msg import SimonCmdAction, SimonCmdResult

def execute_cb(goal):
    rospy.loginfo('Stub motor: executing gesture %s (says=%s)',
                  goal.gesture_name, goal.simon_says)
    time.sleep(0.5)
    server.set_succeeded(SimonCmdResult(executed=True))

rospy.init_node('simon_stub_server')
server = actionlib.SimpleActionServer('/simon_cmd',
                                      SimonCmdAction,
                                      execute_cb, False)
server.start()
rospy.spin()
