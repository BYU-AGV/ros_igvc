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
    try: plt.pause(0.05)
    except Exception: pass

    plt.title('Global Map')

    x,y,arrows = g_map.get_scatter_data()

    plt.scatter(x,y, s=50, color='blue')

    for (a,b) in arrows:
        plt.gca().annotate("", xy=a, xytext=b, arrowprops=dict(arrowstyle="->"))
        # plt.arrow(*a, width=.002, head_width=.55, color='black')

def show():
    plt.show()

if __name__ == '__main__':
    pass

