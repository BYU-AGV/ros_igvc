#!/usr/bin/env python

'''
Description: This is a script to test the pathfinding extension 
Last Modified: 25 Feb 2019
Author: Isaac Draper
'''

import pathfinding

nodes = [
            [0,0],
            [1,0],
            [2,0]
        ]

edges = [
            [0,0,1],
            [1,1,0],
            [0,0,0]
        ]

print pathfinding.search(nodes, edges, [0,0], [1,0], algorithm='bfs', cost_function='man')
print pathfinding.search(nodes, edges, [0,0], [1,0])
print pathfinding.search(nodes, edges, [1,0], [2,0], algorithm='bfs', cost_function='man')
print pathfinding.search(nodes, edges, [1,0], [2,0])

