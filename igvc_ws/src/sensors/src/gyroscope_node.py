#!/usr/bin/env python


import ros_api as ros
from ros_api import println

import custom_msgs.msg as msgs

gyroscope_data = None

def callback(msg_data):
    global gyroscope_data
    gyroscope_data = msg_data
    println("Data {}".format(msg_data))


if __name__ == '__main__':
    sub = ros.ROS_Subscriber('gyroscope_node', 'gyroscope_sensor', msgs.gyroscope, call=callback)
    ros.ros_spin()

