#!/usr/bin/env python

'''
    Discription: GPS ROS node, keeps track of the last known GPS location
'''

import ros_api as ros
from ros_api import println
import custom_msgs.msg as msgs
import custom_msgs.srv as srv
import rospy
import math

last_gps_data = None


'''
    This is ther service proxy, function is triggered when a node calls the server proxy through ROS

    agrs: requet - it is the request .srv message
'''
def location_to_waypoint(request):
    dist = waypoint_to_waypoint(last_gps_data, request)
    return srv.location_to_waypointResponse(0, 0, 0, dist, 0)


'''
    Calculates the distance between two waypoints. Uses the haversine forumla to calculate the shortest distance between the two waypoints

    args: way1 - a waypoint of the type msgs.gps
          way2 - second waypoint of the same type
'''
def waypoint_to_waypoint(way1, way2):
    if way1 == None or way2 == None:
        return 0
    # Do some fancy calculations to figure out stuff
    r = 6378137 # earth circumfrance in km
    dlat = ((way2.latitude * math.pi) / 180.0) - ((way1.latitude * math.pi) / 180.0)
    dlon = ((way2.longitude * math.pi) / 180.0) - ((way1.longitude * math.pi) / 180.0)
    hav = math.sin(dlat / 2.0) * math.sin(dlat / 2.0) + math.cos(way1.latitude * math.pi / 180.0) * math.cos(way2.latitude * math.pi / 180.0) * math.sin(dlon / 2.0) * math.sin(dlon / 2.0)
    c = 2.0 * math.atan2(math.sqrt(hav), math.sqrt(1.0 - hav))
    return r * c


'''
    This creates the service proxy that will be triggered when called through ROS
'''
def create_service_proxy():
        service = rospy.Service('location_to_waypoint', srv.location_to_waypoint, location_to_waypoint)

'''
    This is the callback for the subscriber, updates the current gps location

    args: gps_data - this is the message object from the even
                 is the same type that the subscriber was set up to recieve
'''
def callback(gps_data):
    global last_gps_data
    println("Distance: " + str(location_to_waypoint(gps_data).distance))
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
