#!/usr/bin/env python

import ros_api as ros
from ros_api import println
import rospy
import custom_msgs.msg as msgs


def callback(data):
    println("Data: {} ".format(data))


def start_listening():
    sub = ros.ROS_Subscriber('gps_node', 'gps_location', msgs.gps, callback)
    rospy.spin()



if __name__ == '__main__':
    start_listening()
