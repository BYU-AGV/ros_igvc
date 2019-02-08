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
    num = 0
    
    def __init__(self, latitude, longitude, radius, parent):
        self.lat = latitude
        self.lon = longitude
        self.r = radius

        if type(parent) != type(self) and parent != None:
            raise TypeError('Unable to create a node with a non-node parent')
        self.parents = [parent]

        Node.num += 1

    def __repr__(self):
        return __str__()
    def __str__(self):
        return 'Node: ({},{}) Parents: {}'.format(self.lat, self.lon, len(self.parents))

    def add_parent(self, parent):
        if parent not in self.parents:
            self.parents.append(parent)

    def get_latitude(self):
        return self.lat

    def get_longitude(self):
        return self.lon

    def get_parents(self):
        return self.parents

    def get_radius(self):
        return self.r

