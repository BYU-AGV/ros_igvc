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
from visualize import display_graph

if __name__ == '__main__':
    g_map = Map((14, 12))
    display_graph(None)
