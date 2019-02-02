#!/usr/bin/env python


'''
    Description: Reads/Subsrcibes imu data, process the data to determine current velocity and delta distance
    Date Modified: 1 Feb 2019
    Author: Ben Brenkman
'''


import ros_api as ros
from ros_api import println

import custom_msgs.msg as msgs

imu_data = None # last known imu data in JSON format

velocity_x = 0.0
velocity_y = 0.0
velocity_z = 0.0
velocity_magnitude = 0.0

distance_x = 0.0
distance_y = 0.0
distance_z = 0.0
distance_magnitude = 0.0

velocity_pub = None
distance_pub = None


'''
    Calucates velocity in x, y, z directions and the magnitude
    
    args: data - data object of type msgs.imu
'''
def calculate_velocity(data):
    return data.x * (data.duration / 1000.0), 
           data.y * (data.duration / 1000.0), 
           data.z * (data.duration / 1000.0), 
           ((velocity_x ** 2) + (velocity_y ** 2) + (velocity_z ** 2)) ** (1/2.0)



'''
    Calcuates change in distance i.e. delta distance

    args: data - data of type msgs.imu
'''
def calculate_distance(data):
    return data.x * ((data.duration / 1000.0) ** 2.0), 
           data.y * ((data.duration / 1000.0) ** 2.0), 
           data.z * ((data.duration / 1000.0) ** 2.0), 
           ((distance_x ** 2) + (distance_y ** 2) + (distance_z ** 2)) ** (1/2.0)



'''
    Standard callback for the subscriber. Calculates velocity and delta distance and stores data. Also publishes data to ROS

    args: msg_data - data object of type msgs.imu
'''
def callback(msg_data):
    global imu_data, velocity_x, velocity_y, velocity_z, velocity_magnitude, velocity_pub, distance_pub
    imu_data = msg_data
    println("Data {}".format(msg_data))
    if msg_data.type == 'imu':
        velocity_x, velocity_y, velocity_z, velocity_magnitude = calculate_velocity(msg_data)
        distance_x, distance_y, distance_z, distance_magnitude = calculate_distance(msg_data)

        velocity_pub.send(velocity_x, velocity_y, velocity_z, velocity_magnitude)
        distance_pub.send(distance_x, distance_y, distance_z, distance_magnitude)
        println("Velocity: x: " + str(velocity_x) + ', y: ' + str(velocity_y) + ', z: ' + str(velocity_z) + ', mag: ' + str(velocity_magnitude))
        println("Distance: x: " + str(distance_x) + ', y: ' + str(distance_y) + ', z: ' + str(distance_z) + ', mag: ' + str(distance_magnitude))



'''
    Main function. Sets up everything including the subscribers and publishers
'''
if __name__ == '__main__':
    ros.init_node('imu_node')
    sub = ros.Subscriber('sensor_imu_raw', msgs.imu, callback)
    velocity_pub = ros.Publisher('sesnor_imu_velocity', msgs.velocity)
    distance_pub = ros.Publisher('sesnor_imu_delta_distance', msgs.imu_distance)
    ros.spin()
