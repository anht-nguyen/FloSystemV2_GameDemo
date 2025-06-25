#!/usr/bin/env python3
import rospy, sys, termios, tty, select
from flo_core_defs.msg import PoseScore

def getch():
    fd = sys.stdin.fileno(); old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd); ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

rospy.init_node('pose_stub')
pub = rospy.Publisher('/pose_score', PoseScore, queue_size=1)
rate = rospy.Rate(10)
matched = False
print('Press [m] to toggle matched flag, [q] to quit')
while not rospy.is_shutdown():
    if sys.stdin in select.select([sys.stdin],[],[],0)[0]:
        c = getch()
        if c == 'm': matched = not matched
        if c == 'q': break
    pub.publish(PoseScore(matched=matched))
    rate.sleep()
