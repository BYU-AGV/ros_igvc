#!/usr/bin/env python


import ros_api as ros
from ros_api import println

import custom_msgs.msg as msgs

imu_data = None

def callback(msg_data):
    global imu_data
    imu_data = msg_data
    println("Data {}".format(msg_data))


if __name__ == '__main__':
    sub = ros.ROS_Subscriber('imu_node', 'imu_sensor', msgs.imu, call=callback)
    ros.ros_spin()
