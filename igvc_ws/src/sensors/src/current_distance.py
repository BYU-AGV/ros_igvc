#!/usr/bin/env python

import ros_api as ros
from ros_api import println

import custom_msgs.msg as msgs

total_distance_x = 0.0
total_distance_y = 0.0
total_distance_z = 0.0

def callback(imu_delta):
    global total_distance_x, total_distance_y, total_distance_z, total_distance_magnitude
    total_distance_x += imu_delta.delta_x
    total_distance_y += imu_delta.delta_y
    total_distance_z += imu_delta.delta_z
    println('Distance: x: ' + str(total_distance_x) + ', y: ' + str(total_distance_y) + ', z: ' + str(total_distance_z))


if __name__ == '__main__':
    sub = ros.ROS_Subscriber('total_distance', 'imu_delta_distance', msgs.imu_distance, call=callback)
    ros.ros_spin()
