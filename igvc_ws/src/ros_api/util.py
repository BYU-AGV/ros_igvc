#!/usr/bin/env python

'''
This is a script for basic useful utilities.
Last edited: 25 Jan 2019
Author: Isaac
'''

import sys

def println (*msg, **kwargs):
    if 'end' not in kwargs: kwargs['end'] = '\n'
    sys.stdout.write(''.join(map(str, msg)).strip() + kwargs['end'])
    sys.stdout.flush()

