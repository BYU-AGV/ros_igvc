#!/usr/bin/env python


'''
    Description: Compass node
    Date Modified: 1 Feb 2019
    Author: Ben Brenkman
'''

import ros_api as ros
from ros_api import println

import custom_msgs.msg as msgs

compass_data = None # last known compass heading


'''
    Standard callback for the compass subscriber

    args: msg_data - data object of type msgs.compass
'''
def callback(msg_data):
    global compass_data
    compass_data = msg_data
    println("Data {}".format(msg_data))

'''
    Main function, starts up subscribers and publishers
'''
if __name__ == '__main__':
    ros.init_node('comapss_node')
    pub = ros.Subscriber('compass_sensor_raw', msgs.compass, call=callback)
    ros.spin()

