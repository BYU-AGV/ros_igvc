#!/usr/bin/env python

'''
This is a script for basic useful utilities and functions.
Last edited: 25 Jan 2019
Author: Isaac
'''

import sys

import rospy

def println (*msg, **kwargs):
    ''' This is a function that mimicks python3's print

        Args:   *msg - an object to print

        Kwargs: end - what to print at the end of the string (default '\n')

        Returns: None
    '''

    if 'end' not in kwargs: kwargs['end'] = '\n'
    sys.stdout.write(''.join(map(str, msg)).strip() + kwargs['end'])
    sys.stdout.flush()

def ros_spin():
    ''' This is a function to make the python script not exit (for subscribers) 
        
        Args:   None

        Returns:    None
    '''

    rospy.spin()

def ros_is_running():
    ''' Tells if the ros node is running

        Args:   None

        Returns:    True if the node is running, False if not
    '''

    return not rospy.is_shutdown()

def ros_is_shutdown():
    ''' Tells if the ros node is not running

        Args:   None

        Returns:    True if the node is not running, False if it is
    '''

    return rospy.is_shutdown()

def ros_sleep(time):
    ''' Makes the ros_node sleep a given time

        Args:   time - time to sleep in seconds

        Returns:    None
    '''

    rospy.sleep(time)

