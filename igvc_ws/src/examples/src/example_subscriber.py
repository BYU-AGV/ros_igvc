#!/usr/bin/env python

'''
A test script to recieve data using ros_api
Last Updated: 31 Jan 2019
Author: Isaac
'''


import std_msgs.msg as std_msgs
import custom_msgs.msg as msgs

import ros_api as ros
from ros_api import println, spin

def custom_callback(data):
    #println('Recieved custom: {}'.format(data))
    println(ros.msg_to_dict(data))

if __name__ == '__main__':
    println('Starting test revieve')

    ros.init_node('test')
    sub1 = ros.Subscriber( 'test_topic', msgs.coord, call=custom_callback)
    sub2 = ros.Subscriber('another_topic', msgs.coord)
    spin()

    println('Node finished with no errors')


