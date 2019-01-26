#!/usr/bin/env python

'''
A test script to act as a publisher from the ros_api.
Last Updated: 25 Jan 2019
Author: Isaac
'''

import rospy
from geometry_msgs.msg import Twist

import std_msgs.msg as std_msgs
import custom_msgs.msg as msgs

import ros_api as ros
from ros_api import println

if __name__ == '__main__':
    println('Starting test pkg')
    
    handler = ros.ROS_Publisher('test_publisher', 'test_topic', msgs.coord)
    while not rospy.is_shutdown():
        # handler.send(32,10,500)
        handler.send(x=10,y=32,z=40)
        rospy.sleep(1)


    println('Node finished with no errors')
