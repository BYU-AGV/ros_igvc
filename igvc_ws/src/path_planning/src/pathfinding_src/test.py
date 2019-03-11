#!/usr/bin/env python

'''
Description: This is a script to test the pathfinding extension 
Last Modified: 11 Mar 2019
Author: Isaac Draper
'''

import pathfinding
import random

'''
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
'''

i = 1000
n = 30000
for _ in range(n):
    nodes = [[x,0] for x in range(i)]
    edges = [[(1 if random.random() > .95 else 0) for x in range(i)] for _ in range(i)]

    print pathfinding.search(nodes, edges, [0,0], [11,0], algorithm='bfs', cost_function='man')
    print pathfinding.search(nodes, edges, [0,0], [11,0])
    print pathfinding.search(nodes, edges, [1,0], [22,0], algorithm='bfs', cost_function='man')
    print pathfinding.search(nodes, edges, [1,0], [22,0])

