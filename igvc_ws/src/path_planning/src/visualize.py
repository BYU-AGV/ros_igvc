#!/usr/bin/env python

'''
Description: This is a script for visualizing the global map
Last Modified: 7 Feb 2019
Author: Isaac Draper
'''

import std_msgs.msg as std_msgs
import custom_msgs.msg as msgs

import ros_api as ros
from ros_api import println

import matplotlib.pyplot as plt
import numpy as np

def display_graph(g_map):
    plt.title('Global Map')
    x,y = g_map.get_scatter_data()
    plt.scatter(x,y)
    plt.show()
    pass

if __name__ == '__main__':
    pass

