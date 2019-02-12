#!/usr/bin/env python

'''
Description: This is a class to represent a 2-d map (top down) of known traversible space
Last Modified: 11 Feb 2019
Author: Isaac Draper
'''

import std_msgs.msg as std_msgs
import custom_msgs.msg as msgs

import ros_api as ros
from ros_api import println

import numpy as np
from node import Node
import math
import pickle as pk

class Map(object):
    def __init__(self, latitude, longitude, radius):
        ''' Constructor for a Map object

            Args:   latitude - the gps latitude for the initial node
                    longitude - the gps longitude for the initial node
                    radius - the gps accuracy for the initial node in meters

            Returns:    a Map object
        '''

        self.nodes = []

        self.last_node = Node(latitude, longitude, radius, None)
        self.nodes.append(self.last_node)

    def add_node(self, latitude, longitude, radius):
        ''' Adds a node if that space hasn't already been added

            Args:   latitude - the gps latitude location
                    longitude - the gps longitude location
                    radius - the gps accuracy in meters

            Returns:    a Node object. This will either be a new node or one already traversed
        '''

        node = self.location_within_node(latitude, longitude)
        if node == None:
            node = Node(latitude, longitude, radius, self.last_node)
            self.nodes.append(node)

        self.last_node = node
        return node

    def location_within_node(self, latitude, longitude):
        ''' Checks whether a location already has a node with that location

            Args:   latitude - the gps latitude location
                    longitude - the gps longitude location

            Returns:    the Node at that location, None if one does not exist
        '''

        for n in self.nodes:
            if self.gps_dist_in_meters(latitude, longitude, n.get_latitude(), n.get_longitude()) <= n.get_radius():
                n.add_parent(self.last_node)
                return n

        return None

    def eculid_dist(self, x1, y1, x2, y2):
        ''' Returns the eculidean distance between two eculidean points

            Args:   x1 - point 1's x position
                    y1 - point 1's y position
                    x2 - point 2's x position
                    y2 - point 2's y position

            Returns:    A double representing the distance between points
        '''

        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    def eculid_dist_squared(self, x1, y1, x2, y2):
        ''' Returns the eculidean distance between two points

            Args:   x1 - point 1's x position
                    y1 - point 1's y position
                    x2 - point 2's x position
                    y2 - point 2's y position

            Returns:    A double representing the distance between points
        '''

        return (x2-x1)**2 + (y2-y1)**2
    
    def gps_dist_in_meters(self, lat1, lon1, lat2, lon2):
        ''' Returns the distance in meters between two gps locations

            Args:   lat1 - gps 1's latitude coordinate 
                    lon1 - gps 1's longitude coordinate 
                    lat2 - gps 2's latitude coordinate 
                    lon2 - gps 2's longitude coordinate 

            Returns:    A double representing the distance between coordinates
        '''

        R = 6378137

        d_lat = math.radians(lat2-lat1)
        d_lon = math.radians(lon2-lon1)

        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)

        a = math.sin(d_lat/2) * math.sin(d_lat/2) + \
            math.sin(d_lon/2) * math.sin(d_lon/2) * \
            math.cos(lat1) * math.cos(lat2)

        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    def pickle(self, path):
        ''' Saves the current Map as a pickled file

            Args:   path - the file path and name to save
        '''

        with open(path, 'w') as f:
            pk.dump(self, f)

    def get_scatter_data(self):
        ''' Returns nodes in a data format for a scatter plot '''

        x = np.zeros(len(self.nodes))
        y = np.zeros(len(self.nodes))
        s = np.zeros(len(self.nodes))

        arrows = []
        for i,n in enumerate(self.nodes):
            x_ = n.get_latitude()
            y_ = n.get_longitude()
            r = n.get_radius()

            x[i] = x_
            y[i] = y_
            s[i] = n.get_radius()

            for p in n.get_parents():
                if p == None: continue

                arrows.append(((x_,y_),(p.get_latitude(),p.get_longitude())))

        return x,y,s,arrows

    def get_last_point_scatter(self):
        x_ = self.last_node.get_latitude()
        y_ = self.last_node.get_longitude()
        r = self.last_node.get_radius()
        return x_,y_,r

