#!/usr/bin/env python

'''
Description: This is a script for visualizing the global map
Last Modified: 11 Feb 2019
Author: Isaac Draper
'''

import std_msgs.msg as std_msgs
import custom_msgs.msg as msgs

import ros_api as ros
from ros_api import println

import matplotlib.pyplot as plt
import numpy as np
import math

def display_graph(g_map):
    ''' This function displays a global map object continuously

        Args:   g_map - a Map object
    '''

    try: plt.pause(0.05)
    except Exception: pass

    plt.cla()
    plt.title('Global Map')
    plt.gca().set_aspect('equal')
    plt.gca().autoscale(tight=False)

    x,y,s,arrows = g_map.get_scatter_data()

    # cm = plt.cm.get_cmap('Blues')
    c = np.linspace(0, 1, len(x))

    plt.scatter(x,y, s=scale_sizes(g_map, s), c=c)
    
    px,py,ps = g_map.get_last_point_scatter()
    plt.scatter(px,py, s=scale_sizes(g_map, ps), color='red')

    for (a,b) in arrows:
        plt.gca().annotate("", xy=a, xytext=b, arrowprops=dict(arrowstyle="->"))

def meters_to_angle(m):
    return 100

def scale_sizes(g_map, sizes):
    l_xlim,r_xlim = plt.xlim()
    l_ylim,r_ylim = plt.ylim()

    d_xlim = r_xlim - l_xlim
    d_ylim = r_ylim - l_ylim

    bbox = plt.gca().get_window_extent().transformed(plt.gcf().dpi_scale_trans.inverted())
    width, height = bbox.width, bbox.height
    width *= plt.gcf().dpi
    height *= plt.gcf().dpi

    md_xlim = g_map.gps_dist_in_meters(l_xlim,0,r_xlim,0)
    md_ylim = g_map.gps_dist_in_meters(0,l_ylim,0,r_ylim)

    ratio = md_xlim / width
    ratio = width / md_xlim

    println(ratio)
    println()

    try: sizes[0]
    except TypeError: return (math.pi*(sizes)**2)*ratio

    new_s = []
    for s in sizes:
        # new_s.append(math.pi*(s*ratio)**2)
        new_s.append((math.pi*s**2)*ratio)

    return new_s 

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

