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

import numpy as np
from node import Node

class Map(object):
    def __init__(self, current_pos):
        self.nodes = []

        lat,lon = current_pos
        self.last_node = Node(lat, lon, None)
        self.nodes.append(self.last_node)

    def add_node(self, latitude, longitude):
        node = Node(latitude, longitude, self.last_node)
        self.nodes.append(node)
        self.last_node = node

    def get_scatter_data(self):
        x = np.zeros(len(self.nodes))
        y = np.zeros(len(self.nodes))

        for i,n in enumerate(self.nodes):
            x[i] = n.get_latitude()
            y[i] = n.get_longitude()

        return x,y


