#!/usr/bin/env python

'''
Discription: 
Last Modified: 31 Jan 2019
Author: Isaac Draper 
'''

import ros_api as ros
from ros_api import println
import custom_msgs.msg as msgs


if __name__ == '__main__':
    ros.init_node('data_manager')

    ros.ROS_Subscriber('
    
