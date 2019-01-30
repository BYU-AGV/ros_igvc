#!/usr/bin/env python

'''
Discription: ROS node that recieves sensor information via a micro server
Last Modified: 30 Jan 2019
Author: Ben Brenkman
'''

import socket
import fcntl
import struct
import os
import json

import rospy
import ros_api as ros
from ros_api import println

import custom_msgs.msg as msgs
import std_msgs as std_msgs

from flask import Flask, request, abort, Response, jsonify

app = Flask(__name__)

# Varialbes go here
publisher = None # this is the publisher
gps_location = None # stores the last known GPS coordinates
acceleration = None # stores last known IMU data


'''
This is the usage tag, run this to get a list of url discriptions to use
'''
@app.route('/webhook')
def webhook():
    return "Webhooks :)"


''' This is the acclerometor webhook, updates and gets information about the IMU
To use this webhook you can submit either an GET or POST method
GET:
    If you submit a GET method, you can call the URL as is, there is no need to
    add any arguments. The method will return the last known IMU data as a JSON
    string in the same format as used by ROS. For more information about the 
    strucutre of the message look at the custom messsages reference page on the 
    wiki

POST:
    The POST method is used to update the publish an IMU message to ROS and other
    nodes.
    To use submit the URL as is using POST, for the body inclue a JSON string that
    is in the same format as the custom accelerometer message used for ROS. For 
    more information about the structure of the message look at the custom message
    reference page on the project wiki
'''
@app.route('/webhook/accelerometer', methods=['GET', 'POST'])
def accel():
    global acceleration
    if request.method == 'POST':
        data = request.data
        json_data = json.loads(data)
        acceleration = json_data
        rospy.loginfo('/webhook/accelerometer{' +  'type: ' + str(json_data['type']) + ', x: ' + str(json_data['x']) + ', y: ' + str(json_data['y']) + ', z: ' + str(json_data['z']) + '}')
        return True
    elif request.method == 'GET':
        return jsonify(acceleration)


''' This is the GPS webhook, updates and gets the last known GPS location about the
    To use this webhook you can submit either an GET or a POST method.

GET:
    If you want to submit a GET method, you can submit the URL as is. The GET method
    will return the last known GPS location as a JSON string using the same format 
    as we use in the custom message.

POST:
    Submit the URL as is with the body as a JSON string in the format of the custom 
    GPS message used in ROS to update the GPS location. We will then publish the 
    message to ROS on the topic 'gps_location'
'''
@app.route('/webhook/gps', methods=['GET', 'POST'])
def gps():
    global gps_location
    if request.method == 'POST':
        json_data = json.loads(request.data)
        gps_location = json_data
        rospy.loginfo('/webhook/gps{' + 'latitude: ' +  str(json_data['latitude']) + ', longitude: ' + str(json_data['longitude']) + ', altitude: ' + str(json_data['altitude']) + ', accuracy: ' + str(json_data['accuracy']) + ' speed: ' + str(json_data['speed']) + ', speed_accuracy: ' +  str(json_data['speed_accuracy']) + '}')
        
    if publisher is not None:
        publisher.send(ros.json_to_msg(request.data, msgs.gps))

        return True
    else:
        return jsonify(gps_location)



''' 
Ping function, this is for establishing a cconnection to the micro server. Returns a 
JSON string with status as success. Only GET method allowed
'''
@app.route('/ping')
def ping():
    return True


'''
This initializes the ROS node and sets up publishers
'''
def start_ros():
    global publisher
    publisher = ros.ROS_Publisher('gps_publisher', 'gps_location', msgs.gps)


'''
This function returns a string contianing ip address information
'''
def get_inet_address():
    f = os.popen('ifconfig ens33 | grep "inet [0-9]\{1,3\}\.[0-9]\{1,3\}\.[0-9]\{1,3\}\.[0-9]\{1,3\}"')
    return f.read()


if __name__ == '__main__':
    print "Running on: " + get_inet_address()
    start_ros()
    app.run(host='0.0.0.0')
