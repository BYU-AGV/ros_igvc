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
    def __init__(self, latitude, longitude):
        self.nodes = []

        self.last_node = Node(latitude, longitude, None)
        self.nodes.append(self.last_node)

    def add_node(self, latitude, longitude):
        node = Node(latitude, longitude, self.last_node)
        self.nodes.append(node)
        self.last_node = node

        return node

    def get_scatter_data(self):
        x = np.zeros(len(self.nodes))
        y = np.zeros(len(self.nodes))

        arrows = []
        for i,n in enumerate(self.nodes):
            x_ = n.get_latitude()
            y_ = n.get_longitude()

            x[i] = x_
            y[i] = y_

            for p in n.get_parents():
                if p == None: continue

                arrows.append(((x_,y_),(p.get_latitude(),p.get_longitude())))

        return x,y,arrows


