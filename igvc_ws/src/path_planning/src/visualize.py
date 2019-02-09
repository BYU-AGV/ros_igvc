#!/usr/bin/env python

'''
Description: This is a script for visualizing the global map
Last Modified: 8 Feb 2019
Author: Isaac Draper
'''

import std_msgs.msg as std_msgs
import custom_msgs.msg as msgs

import ros_api as ros
from ros_api import println

import matplotlib.pyplot as plt
import numpy as np

def display_graph(g_map):
    ''' This function displays a global map object continuously

        Args:   g_map - a Map object
    '''

    try: plt.pause(0.05)
    except Exception: pass

    plt.cla()
    plt.title('Global Map')
    plt.gca().set_aspect('equal')

    x,y,s,arrows = g_map.get_scatter_data()

    # cm = plt.cm.get_cmap('Blues')
    c = np.linspace(0, 1, len(x))

    plt.scatter(x,y, s=scale_up(s), c=c)
    
    px,py,ps = g_map.get_last_point_scatter()
    plt.scatter(px,py, s=scale_up(ps), color='red')

    for (a,b) in arrows:
        plt.gca().annotate("", xy=a, xytext=b, arrowprops=dict(arrowstyle="->"))

def scale_up(obj, scale=1000000000):
    try: return map(lambda x : x*scale, obj)
    except TypeError: return obj*scale

def show():
    ''' Wrapper for matplotlib plt.show() '''

    plt.ion()
    plt.show()

def close():
    ''' Wrapper for matplotlib plt.close() '''

    plt.close()

if __name__ == '__main__':
    ''' If this is called from main, nothing should happen '''

    pass

