#!/usr/bin/env python

'''
Description: This is a test script to simulate gps data (generates random data)
Last Modified: 7 Feb 2019
Author: Isaac Draper
'''

import custom_msgs.msg as msgs

import ros_api as ros

import random

if __name__ == '__main__':
    ros.init_node('fake_gps')
    pub = ros.Publisher('test_gps', msgs.gps)

    lat = random.random()*10
    lon = random.random()*10

    while ros.is_running():
        n_lat = random.random()*10
        n_lon = random.random()*10

        lat = random.choice((lat+n_lat,lat-n_lat))
        lon = random.choice((lon+n_lon,lon-n_lon))
        acc = random.random()*5 + 1

        pub.send(   latitude=lat, \
                    longitude=lon, \
                    altitude=0, \
                    accuracy=acc, \
                    speed=0, \
                    speed_accuracy=0
                )

        ros.sleep(1)
        
