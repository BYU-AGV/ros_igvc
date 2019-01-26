#!/usr/bin/env python

'''
A test script to recieve data using ros_api
Last Updated: 25 Jan 2019
Author: Isaac
'''


import std_msgs.msg as msg

import ros_api as ros
from ros_api import println

import rospy

def custom_callback(data):
    println('Recieved custom: {}'.format(data.data))

if __name__ == '__main__':
    println('Starting test revieve')

    handler = ros.ROS_Subscriber('other_svr', 'test_topic', msg.Int32, call=custom_callback)
    rospy.spin()

    println('Node finished with no errors')

