#!/usr/bin/env python

'''
    Discription: GPS ROS node, keeps track of the last known GPS location
'''

import ros_api as ros
from ros_api import println
import custom_msgs.msg as msgs
import custom_msgs.srv as srv
import rospy

last_gps_data = None


def location_to_waypoint(request):
    if last_gps_data = None:
        return srv.location_to_waypointResponse(0, 0, 0, 0, 0)
    # Do some fancy calculations to figure out stuff
    return srv.location_to_waypointResponse(0, 0, 0, 0, 0)


def create_service_proxy():
        service = rospy.Service('location_to_waypoint', srv.location_to_waypoint, location_to_waypoint)

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



if __name__ == '__main__':
    start_listening()
    create_service_proxy()
    ros.spin()
