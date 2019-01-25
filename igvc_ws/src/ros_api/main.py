#!/usr/bin/env python

'''
This is the core ros module that external users will use to call ROS functions
Last Updated: 25 Jan 2019
Author: Isaac
'''

import sys

import rospy
import std_msgs.msg as msg

def println (*msg, **kwargs):
    if 'end' not in kwargs: kwargs['end'] = '\n'
    sys.stdout.write(''.join(map(str, msg)).strip() + kwargs['end'])
    sys.stdout.flush()

class ROS_Handler(object):
    def __init__(self, svr_name, topic, msg_type, rate):
        self.svr_name = svr_name
        self.topic = topic
        self.msg_type = msg_type

        rospy.init_node(self.svr_name)
        self.rate = rospy.Rate(rate)

    def set_rate(self, r):
        self.rate = rospy.Rate(r)


class ROS_Publisher(ROS_Handler):
    def __init__(self, svr_name, topic, msg_type, q_size=10, rate=10):
        super(ROS_Publisher,self).__init__(svr_name, topic, msg_type, rate)
        self.pub = rospy.Publisher(self.topic, self.msg_type, queue_size=q_size)

    def send(self, arg):
        self.pub.publish(arg)
        println('Sent: {}'.format(arg))

class ROS_Subscriber(ROS_Handler):
    def __init__(self, svr_name, topic, msg_type, call=None, rate=10):
        super(ROS_Subscriber,self).__init__(svr_name, topic, msg_type, rate)
        self.callback = self.default_callback if call == None else call
        self.sub = rospy.Subscriber(self.topic, self.msg_type, self.callback)

    def default_callback(self, data):
        println('Recieved: {}'.format(data.data))

    def new_callback(self, call):
        self.sub.unregister()
        self.callback = call
        self.sub = rospy.Subscriber(self.topic, self.msg_type, self.callback)

    def new_topic(self, topic):
        self.sub.unregister()
        self.topic = topic
        self.sub = rospy.Subscriber(self.topic, self.msg_type, self.callback)







