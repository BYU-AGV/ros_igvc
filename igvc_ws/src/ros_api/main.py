#!/usr/bin/env python

'''
This is the core ros module that wraps key ROS functions that external users will use to call ROS functions
Last Updated: 25 Jan 2019
Author: Isaac
'''

from util import println

import rospy
import std_msgs.msg as msg

class ROS_Handler(object):
    '''
        This is a root class to initialize a node for ROS communicaion.
        It typically should not be created, rather is a base class for inheritance.
    '''

    def __init__(self, svr_name, topic, msg_type, rate):
        ''' This is the ROS_Handler constructor

            Args:   svr_name - the name of the node to initialize
                    topic - the topic name (this must be the same for the subscriber and publisher for communication)
                    msg_type - the ROS message being passed
                    rate - number of times per second the node is run by ROS

            Returns:    A ROS_Handler object
        '''

        self.svr_name = svr_name
        self.topic = topic
        self.msg_type = msg_type

        rospy.init_node(self.svr_name)
        self.rate = rospy.Rate(rate)

    def set_rate(self, r):
        ''' Sets how often the ROS node updates per second

            Args:   r - the new rate
        '''

        self.rate = rospy.Rate(r)


class ROS_Publisher(ROS_Handler):
    '''
        A basic wrapper for a ROS publisher.
    '''

    def __init__(self, svr_name, topic, msg_type, q_size=10, rate=10):
        ''' This is the ROS_Publisher constructor

            Args:   see ROS_Handler constructor
                    q_size - the queue size, set this much higher for anthing other than single data types

            Returns: A ROS_Publisher object
        '''
        
        super(ROS_Publisher,self).__init__(svr_name, topic, msg_type, rate)
        self.pub = rospy.Publisher(self.topic, self.msg_type, queue_size=q_size)

    def send(self, arg):
        ''' Publishes data to the topic when created

            Args:   arg - the data to be passed, must match the format of self.msg_type

            Returns:    None
        '''

        self.pub.publish(arg)
        println('Sent: {}'.format(arg))

class ROS_Subscriber(ROS_Handler):
    '''
        A basic wrapper for a ROS subscriber.
    '''

    def __init__(self, svr_name, topic, msg_type, call=None, rate=10):
        ''' This is the ROS_Subscriber constructor

            Args:   see ROS_Handler constructor
                    call = A function to execute when data is recieved, default simply prints the data

            Returns: A ROS_Subscriber object
        '''

        super(ROS_Subscriber,self).__init__(svr_name, topic, msg_type, rate)
        self.callback = self.default_callback if call == None else call
        self.sub = rospy.Subscriber(self.topic, self.msg_type, self.callback)

    def default_callback(self, data):
        ''' This is a default callback function that simply prints out the data recieved

            Args:   data - an event object from ROS

            Returns: None
        '''

        println('Recieved: {}'.format(data.data))

    def new_callback(self, call):
        ''' Re-creates the subscriber with a different callback function

            Args:   call = A function to execute when data is recieved, default simply prints the data

            Returns:    None
        '''

        self.sub.unregister()
        self.callback = call
        self.sub = rospy.Subscriber(self.topic, self.msg_type, self.callback)

    def new_topic(self, topic):
        ''' Re-creates the subscriber with a different topic (channel to listen to)

            Args:   topic - the topic name (this must be the same for the subscriber and publisher for communication)

            Returns:    None
        '''

        self.sub.unregister()
        self.topic = topic
        self.sub = rospy.Subscriber(self.topic, self.msg_type, self.callback)







