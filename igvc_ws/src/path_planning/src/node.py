#!/usr/bin/env python

'''
Description: This is a class to represent a single traversible location
Last Modified: 7 Feb 2019
Author: Isaac Draper
'''

import std_msgs.msg as std_msgs
import custom_msgs.msg as msgs

import ros_api as ros
from ros_api import println

class Node(object):
    def __init__(self, latitude, longitude, parent):
        self.lat = latitude
        self.lon = longitude

        if type(parent) != type(self) and parent != None:
            raise TypeError('Unable to create a node with a non-node parent')
        self.parents = [parent]

