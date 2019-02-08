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
import math

class Map(object):
    def __init__(self, latitude, longitude, radius):
        self.nodes = []

        self.last_node = Node(latitude, longitude, radius, None)
        self.nodes.append(self.last_node)

    def add_node(self, latitude, longitude, radius):
        node = self.location_within_node(latitude, longitude)
        if node == None:
            node = Node(latitude, longitude, radius, self.last_node)
            self.nodes.append(node)

        self.last_node = node
        return node

    def location_within_node(self, latitude, longitude):
        for n in self.nodes:
            if self.eculid_dist(latitude, longitude, n.get_latitude(), n.get_longitude()) <= n.get_radius():
                println('within space')
                n.add_parent(self.last_node)
                return n
        return None

    def eculid_dist(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    def get_scatter_data(self):
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
            s[i] = 3.14*r*r

            for p in n.get_parents():
                if p == None: continue

                arrows.append(((x_,y_),(p.get_latitude(),p.get_longitude())))

        return x,y,s,arrows


