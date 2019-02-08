#!/usr/bin/env python

'''
Description: This is a class to represent a 2-d map (top down) of known traversible space
Last Modified: 7 Feb 2019
Author: Isaac Draper
'''

import std_msgs.msg as std_msgs
import custom_msgs.msg as msgs

import ros_api as ros
from ros_api import println

class Map(object):
    def __init__(self, width=100, height=100, scale=1):
        self.width = width
        self.height = height
        self.scale = 1

