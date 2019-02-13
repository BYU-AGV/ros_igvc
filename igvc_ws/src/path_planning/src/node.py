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
    ''' A representation of a location in 2-d space, with a range '''

    num = 0     # to keep track of how many nodes there are
    
    def __init__(self, latitude, longitude, radius, parent):
        ''' Constructor for a Node object

            Args:   latitude - a gps latitude location
                    longitude - a gps longitude location
                    radius - the accuracy of the gps reading
                    parent - the previous Node traveled from

            Returns:    A Node object
        '''

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
        ''' Adds a Node as a parent if not already added

            Args:   parent - the Node to add

            Returns:    True if added, False if not
        '''

        if parent not in self.parents:
            self.parents.append(parent)
            return True
        else:
            return False

    def get_latitude(self):
        ''' Returns the latitude of this node '''

        return self.lat

    def get_longitude(self):
        ''' Returns the longitude of this node '''

        return self.lon

    def get_parents(self):
        ''' Returns the parents of this node '''

        return self.parents

    def get_radius(self):
        ''' Returns the radius (accuracy) of this node '''

        return self.r

