#!/usr/bin/env python

'''
Description: This is the main class to call for path navigation
Last Modified: 7 Feb 2019
Author: Isaac Draper
'''

import std_msgs.msg as std_msgs
import custom_msgs.msg as msgs

import ros_api as ros
from ros_api import println

from global_map import Map
from visualize import display_graph, show

g_map = None

def callback(msg):
    global g_map

    data = ros.msg_to_dict(msg)
    
    if g_map == None:
        g_map = Map(data['latitude'],data['longitude'])
    else:
        g_map.add_node(data['latitude'],data['longitude'])

    display_graph(g_map)

if __name__ == '__main__':
    ros.init_node('pathing_node')
    sub = ros.Subscriber('test_gps', msgs.gps, call=callback)
    ros.spin()

