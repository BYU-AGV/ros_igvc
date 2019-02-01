#!/usr/bin/env python


'''
    Description: This is the main gyrcoscope node
    Date Modified: 1 Feb 2019
    Author: Ben Brenkman
'''


import ros_api as ros
from ros_api import println
import rospy

import custom_msgs.msg as msgs

gyroscope_data = None # last known gyroscope data


'''
    Standard callback function, records data and prints

    args: msg_data - data object of type msgs.gyroscope
'''
def callback(msg_data):
    global gyroscope_data
    gyroscope_data = msg_data
    println("Data {}".format(msg_data))


'''
    Main method, sets up the subscriber
'''
if __name__ == '__main__':
    ros.init_node('gyroscope_node')
    sub = ros.Subscriber('sensor_gyroscope_raw', msgs.gyroscope, call=callback)
    rospy.spin()

