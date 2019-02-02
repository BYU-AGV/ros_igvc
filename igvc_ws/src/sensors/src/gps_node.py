#!/usr/bin/env python

'''
    Discription: GPS ROS node, keeps track of the last known GPS location
'''

import ros_api as ros
from ros_api import println
import custom_msgs.msg as msgs

last_gps_data = None

'''
    This is the callback for the subscriber, updates the current gps location

    args: gps_data - this is the message object from the even
                 is the same type that the subscriber was set up to recieve
'''
def callback(gps_data):
    global last_gps_data
    last_gps_data = gps_data
    println("Data: {} ".format(gps_data))


'''
    This starts up the node, sets up a subscriber for getting last know gps location
'''
def start_listening():
    ros.init_node('gps_node')
    sub = ros.Subscriber('sensor_gps_raw', msgs.gps, callback)
    ros.spin()



if __name__ == '__main__':
    start_listening()
