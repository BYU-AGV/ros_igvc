#!/usr/bin/env python
'''
    Discription: GPS ROS node, keeps track of the last known GPS location
    Date Modified: 12 Feb 2019
    Author: Ben Brenkman
'''

import ros_api as ros
from ros_api import println
import custom_msgs.msg as msgs
import custom_msgs.srv as srv
import rospy # TODO once ros_api can handle services switch to use ros_api
import math

last_gps_data = None # last gps location, TODO replace when we have a gps sensor, put into an object


'''
    This is ther service proxy, function is triggered when a node calls the server proxy through ROS

    agrs: requet - it is the request .srv message
'''
def location_to_waypoint(request):
    dist = waypoint_to_waypoint_dist(last_gps_data, request)
    head = waypoint_to_waypoint_head(last_gps_data, request)
    x, y, z = dist_head_to_vector(dist, head)
    return srv.location_to_waypointResponse(x, y, z, dist, head)


'''
    This function will convert distance and heading into a vector, pointing from the robot to the waypoint
'''
def dist_head_to_vector(distance, heading):
    x = math.cos(heading * math.pi / 180.0) * distance
    y = math.sin(heading * math.pi / 180.0) * distance
    z = 0 # TODO use the ailtiude to calulate the z
    return x, y, z


'''
    This function will take two waypoints and calculate the forward azimuth or heading. This is the initial bearing, 
    it will point in the direction that will take you in a straight line to the target waypoint.

    args: way1 - a gps waypoint of type msgs.gps
          way2 - the second gps waypoint of the saem type
'''
def waypoint_to_waypoint_head(way1, way2):
    if way1 == None or way2 == None:
        return 0
    x = math.sin(way2.longitude - way1.longitude) * math.cos(way2.latitude)
    y = math.cos(way1.latitude) * math.sin(way2.latitude) - math.sin(way1.latitude) * math.cos(way2.latitude) * math.cos(way2.longitude - way1.longitude)
    bearing = math.atan2(y, x) * 180.0 / math.pi
    return bearing


'''
    Calculates the distance between two waypoints. Uses the haversine forumla to calculate the shortest distance between the two waypoints

    args: way1 - a waypoint of the type msgs.gps
          way2 - second waypoint of the same type
'''
def waypoint_to_waypoint_dist(way1, way2):
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
    heading_data = location_to_waypoint(gps_data)
    println("Change: {} ".format(heading_data))
    last_gps_data = gps_data
    println("Data: {} ".format(gps_data))
    println("")


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
