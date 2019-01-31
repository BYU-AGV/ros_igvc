#!/usr/bin/env python

'''
A test script to act as a publisher from the ros_api.
Last Updated: 31 Jan 2019
Author: Isaac
'''

import std_msgs.msg as std_msgs
import custom_msgs.msg as msgs

import ros_api as ros
from ros_api import println, is_running, sleep

if __name__ == '__main__':
    println('Starting test pkg')
    
    ros.init_node('pub_node')
    pub1 = ros.ROS_Publisher('test_topic', msgs.coord)
    pub2 = ros.ROS_Publisher('another_topic', msgs.coord)

    while is_running():
        # pub1.send(32,10,500)
        # pub1.send(x=10,y=32,z=40)

        m = ros.json_to_msg('{"x":5,"y":2,"z":14}', msgs.coord)
        pub1.send(m)

        pub2.send(32,102,1)

        sleep(1)


    println('Node finished with no errors')
