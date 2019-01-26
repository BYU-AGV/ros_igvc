#!/usr/bin/env python

'''
A test script to act as a publisher from the ros_api.
Last Updated: 25 Jan 2019
Author: Isaac
'''

import rospy
from geometry_msgs.msg import Twist

import std_msgs.msg as msg

import ros_api as ros
from ros_api import println

if __name__ == '__main__':
    println('Starting test pkg')

    handler = ros.ROS_Publisher('test_publisher', 'test_topic', msg.Int32)
    while not rospy.is_shutdown():
        handler.send(32)
        rospy.sleep(1)


    println('Node finished with no errors')
