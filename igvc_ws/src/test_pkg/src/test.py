#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import ros_api as ros
from ros_api import println

'''
class StateMachine:
    def __init__(self):
        rospy.init_node('test_pkg')

        # set rate
        hz = 60 # set the number of times this node is executed a second
        self.rate = rospy.Rate(hz)
        println('Starting test pkg')
    def execute(self):
        println('Entered main loop')
        while not rospy.is_shutdown():
            pass
        println('Exiting main loop')
'''

if __name__ == '__main__':
    # StateMachine = StateMachine()
    # StateMachine.execute()

    handler = ros.ROS_Handler()

    println('Node finished with no errors')
