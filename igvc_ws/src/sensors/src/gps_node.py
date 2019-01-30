#!/usr/bin/env python

import ros_api as ros
from ros_api import println
import custom_msgs.msg as msgs


last_gps_location = None

'''
    This is the callback for the subscriber, updates the current gps location

    args: data - this is the message object from the even
                 is the same type that the subscriber was set up to recieve
'''
def callback(data):
    global last_gps_location
    last_gps_location = data
    println("Data: {} ".format(data))


'''
    This starts up the node, sets up a subscriber for getting last know gps location
'''
def start_listening():
    sub = ros.ROS_Subscriber('gps_node', 'gps_location', msgs.gps, callback)
    ros.ros_spin()



if __name__ == '__main__':
    start_listening()
