#!/usr/bin/env python

'''
Description: This is a script to test the pathfinding extension 
Last Modified: 13 Feb 2019
Author: Isaac Draper
'''

import pathfinding

nodes = [
            [0,0],
            [1,0],
            [2,0]
        ]

weights = [
            [0,0,1],
            [1,0,0],
            [0,1,0]
          ]

print pathfinding.search(nodes, weights, [2,0], [0,0], 'bfs', 'euclid')

