#!/usr/bin/env python


import ros_api as ros
from ros_api import println

import custom_msgs.msg as msgs

compass_data = None

def callback(msg_data):
    global compass_data
    compass_data = msg_data
    println("Data {}".format(msg_data))


if __name__ == '__main__':
    pub = ros.ROS_Subscriber('compass_node', 'compass_sensor', msgs.compass, call=callback)
    ros.ros_spin()

