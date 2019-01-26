#!/usr/bin/env python

'''
This is a script for basic useful utilities and functions.
Last edited: 25 Jan 2019
Author: Isaac
'''

import sys

def println (*msg, **kwargs):
    ''' This is a function that mimicks python3's print

        Args:   *msg - an object to print

        Kwargs: end - what to print at the end of the string (default '\n')

        Returns: None
    '''

    if 'end' not in kwargs: kwargs['end'] = '\n'
    sys.stdout.write(''.join(map(str, msg)).strip() + kwargs['end'])
    sys.stdout.flush()

