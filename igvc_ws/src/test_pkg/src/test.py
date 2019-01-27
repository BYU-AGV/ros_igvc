#!/usr/bin/env python

'''
A test script to act as a publisher from the ros_api.
Last Updated: 25 Jan 2019
Author: Isaac
'''

import std_msgs.msg as std_msgs
import custom_msgs.msg as msgs
# import msg as msgs

import ros_api as ros
from ros_api import println, ros_is_running, ros_sleep

if __name__ == '__main__':
    println('Starting test pkg')

    handler = ros.ROS_Publisher('test_publisher', 'test_topic', msgs.coord)
    while ros_is_running():
        # handler.send(32,10,500)
        handler.send(x=10,y=32,z=40)
        ros_sleep(1)


    println('Node finished with no errors')
