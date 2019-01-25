#!/usr/bin/env python

'''
This is the core ros module that external users will use to call ROS functions
Last Updated: 25 Jan 2019
Author: Isaac
'''

import sys

import rospy

def println (*msg, **kwargs):
    if 'end' not in kwargs: kwargs['end'] = '\n'
    sys.stdout.write(''.join(map(str, msg)).strip() + kwargs['end'])
    sys.stdout.flush()

class ROS_Handler(object):
    def __init__(self):
        rospy.init_node('test_server')
        s = rospy.Service('test_server', run_test, self.test)

    def test(self):
        print ('this is a test function')

