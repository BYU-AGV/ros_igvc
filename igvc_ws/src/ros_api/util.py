#!/usr/bin/env python

'''
This is a script for basic useful utilities and functions.
Last edited: 26 Jan 2019
Author: Isaac
'''

import sys
import json

import rospy
import std_msgs.msg as std_msgs
import custom_msgs.msg as msgs


def println (*args, **kwargs):
    ''' This is a function that mimicks python3's print

        Args:   *args - an object to print

        Kwargs: end - what to print at the end of the string (default '\n')

        Returns: None
    '''

    if 'end' not in kwargs: kwargs['end'] = '\n'
    sys.stdout.write(''.join(map(str, args)).strip() + kwargs['end'])
    sys.stdout.flush()

def is_running():
    ''' Tells if the ros node is running

        Args:   None

        Returns:    True if the node is running, False if not
    '''

    return not rospy.is_shutdown()

def json_to_msg(j_str, msg_type):
    ''' Converts a JSON string to a message object

        Args:   j_str - a JSON formatted string
                msg_type - a message class to convert to

        Returns:    A message object of msg_type
    '''

    return msg_type(**(json.loads(j_str)))

def dict_to_msg(d, msg_type):
    ''' Converts a dictionary to a message object

        Args:   d - a dictionary (no more than depth 1)
                msg_type - a message class to convert to

        Returns:    A message object of msg_type
    '''

    return msg_type(**d)

def msg_to_dict(m):
    ''' Converts a message object to a dictionary
        
        Args:   m - a message object

        Returns:    A dictionary of the format:
                        key - the variable name
                        val - the variables value
    '''

    return {e : getattr(m,e) for e in m.__slots__}

def msg_to_json(m):
    ''' Converts a message object to a JSON formatted string

        Args:   m - a message object

        Returns:    A JSON formatted string of the format:
                        key - the variable name
                        val - the variables value
    '''

    return json.dumps(msg_to_dict(m))

