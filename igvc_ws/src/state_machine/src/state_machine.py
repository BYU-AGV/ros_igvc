#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class StateMachine:
    def __init__(self):
        rospy.init_node('state_machine')

        # set rate
        hz = 60 # set the number of times this node is executed a second
        self.rate = rospy.Rate(hz)
        rospy.loginfo('State Machine running')
    def execute(self):
        rospy.loginfo("Now entering the node loop")
        while not rospy.is_shutdown():


if __name__ == '__main__':
    StateMachine = StateMachine()
    StateMachine.execute()
