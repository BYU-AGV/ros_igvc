#!/usr/bin/env python

'''
Description: This is a script for building the python pathfinding extension 
Last Modified: 13 Feb 2019
Author: Isaac Draper
'''

from distutils.core import setup, Extension

MOD = 'pathfinding'
srcs = ['main.cpp']

setup (name = MOD,
       version = '0.1',
       description = 'This is a module to run pathfinding algorithms',
       ext_modules = [
                Extension (
                    MOD,
                    sources = srcs
                )
           ]
       )

