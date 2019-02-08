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

    def __repr__(self):
        return __str__()
    def __str__(self):
        return 'Node: ({},{}) Parents: {}'.format(self.lat, self.lon, len(self.parents))

    def get_latitude(self):
        return self.lat

    def get_longitude(self):
        return self.lon

    def get_parents(self):
        return self.parents

