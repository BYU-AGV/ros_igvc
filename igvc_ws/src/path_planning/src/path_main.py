#!/usr/bin/env python

'''
Description: This is the main class to call for path navigation
Last Modified: 8 Feb 2019
Author: Isaac Draper
'''

import std_msgs.msg as std_msgs
import custom_msgs.msg as msgs

import ros_api as ros
from ros_api import println

from global_map import Map
from visualize import display_graph, close
import signal
import sys

g_map = None

def exit_nicely(signal, frame):
    ''' This is a callback function to try and make it exit without a bunch of errors when pressing Ctrl-C '''

    close()
    println()
    println('exiting...')
    sys.exit(0)

def callback(msg):
    ''' Callback function when data is received from the gps topic

        Args:   msg - the data in message format from the topic
    '''

    global g_map

    data = ros.msg_to_dict(msg)
    
    if g_map == None:
        g_map = Map(data['latitude'],data['longitude'],data['accuracy'])
    else:
        g_map.add_node(data['latitude'],data['longitude'],data['accuracy'])

    display_graph(g_map)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, exit_nicely)

    ros.init_node('pathing_node')
    sub = ros.Subscriber('test_gps', msgs.gps, call=callback)
    # sub = ros.Subscriber('sensor_gps_raw', msgs.gps, call=callback)

    try: ros.spin()
    except RuntimeError: pass

    # Uncomment this if you want to save the map 
    # if g_map != None:
        # g_map.pickle('src/path_planning/src/test_map.pkl')

