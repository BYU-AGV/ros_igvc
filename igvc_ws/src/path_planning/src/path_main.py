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

if __name__ == '__main__':
    g_map = Map((14, 12))
    node = g_map.add_node(13,11)
    g_map.add_node(9,3)
    g_map.add_node(1,3)

    display_graph(g_map)
    show()
    
