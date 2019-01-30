#!/usr/bin/env python

import ros_api as ros
from ros_api import println
import custom_msgs.msg as msgs


'''
    This is the callback for the subscriber

    args: data - this is the message object from the even
                 is the same type that the subscriber was set up to recieve
'''
def callback(data):
    println("Data: {} ".format(data))


'''
This starts up the node
'''
def start_listening():
    sub = ros.ROS_Subscriber('gps_node', 'gps_location', msgs.gps, callback)
    ros.ros_spin()



if __name__ == '__main__':
    start_listening()
