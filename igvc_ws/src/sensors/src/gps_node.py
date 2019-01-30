#!/usr/bin/env python

'''
    Discription: GPS ROS node, keeps track of the last known GPS location
'''

import ros_api as ros
from ros_api import println
import custom_msgs.msg as msgs
import rospy

last_gps_data = None


'''
    This is the callback for the subscriber

    args: gps_data - this is the message object from the even
                 is the same type that the subscriber was set up to recieve
'''
def callback(gps_data):
    global last_gps_data
    last_gps_data = gps_data
    println("Data: {} ".format(gps_data))


'''
    This starts up the node, both the subscriber and service
'''
def start_listening():
    sub = ros.ROS_Subscriber('gps_node', 'gps_location', msgs.gps, callback)
    ros.ros_spin()



if __name__ == '__main__':
    start_listening()
