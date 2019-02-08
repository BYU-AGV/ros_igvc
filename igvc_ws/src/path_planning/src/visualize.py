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

    x,y,s,arrows = g_map.get_scatter_data()

    cm = plt.cm.get_cmap('Blues')
    c = np.linspace(0, 1, len(x))

    plt.scatter(x,y, s=s, c=c, cmap=cm)

    for (a,b) in arrows:
        plt.gca().annotate("", xy=a, xytext=b, arrowprops=dict(arrowstyle="->"))

def show():
    plt.ion()
    plt.show()

def close():
    plt.close()

if __name__ == '__main__':
    pass

