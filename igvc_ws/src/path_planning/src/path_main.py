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

def callback(msg):
    data = ros.msg_to_dict(msg)
    g_map.add_node(data['latitude'],data['longitude'])
    display_graph(g_map)

if __name__ == '__main__':
    ros.init_node('pathing_node')
    sub = ros.Subscriber('test_gps', msgs.gps, call=callback)

    g_map = Map((14, 12))
    node = g_map.add_node(13,11)
    g_map.add_node(9,3)
    g_map.add_node(1,3)

    display_graph(g_map)
    show()
    
