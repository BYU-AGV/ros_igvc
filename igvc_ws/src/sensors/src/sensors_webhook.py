#!/usr/bin/env python

'''
Discription: ROS node that recieves sensor information via a micro server
Last Modified: 8 Feb 2019
Author: Ben Brenkman
'''

import socket
import fcntl
import struct
import os
import json

import ros_api as ros
from ros_api import println

import custom_msgs.msg as msgs
import std_msgs as std_msgs

from flask import Flask, request, abort, Response, jsonify

app = Flask(__name__)

# Publishers
gps_pub = None # this is the publisher
imu_pub = None
gyroscope_pub = None
compass_pub = None

# Storage variables
gps_location = None # stores the last known GPS coordinates
imu_data = None # stores last known IMU data
gyroscope_data = None # stores last known gyroscope data
compass_data = None # last known compass heading


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
    global imu_data
    if request.method == 'POST':
        data = request.data
        json_data = json.loads(data)
        imu_data = json_data
        ros.loginfo('/webhook/accelerometer{' +  'type: ' + str(json_data['type']) + ', x: ' + str(json_data['x']) + ', y: ' + str(json_data['y']) + ', z: ' + str(json_data['z']) + ', duration: ' + str(json_data['duration']) + '}')
        imu_pub.send(ros.json_to_msg(request.data, msgs.imu))
        return True
    elif request.method == 'GET':
        return jsonify(imu_data)


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
        ros.loginfo('/webhook/gps{' + 'latitude: ' +  str(json_data['latitude']) + ', longitude: ' + str(json_data['longitude']) + ', altitude: ' + str(json_data['altitude']) + ', accuracy: ' + str(json_data['accuracy']) + ' speed: ' + str(json_data['speed']) + ', speed_accuracy: ' +  str(json_data['speed_accuracy']) + '}')
        gps_pub.send(ros.json_to_msg(request.data, msgs.gps))
        return True
    else:
        return jsonify(gps_location)


'''
Compass update and get webhook
GET:
    If you want to submit a GET method, you can submit the URL as is. The GET method
    will return the last known GPS location as a JSON string using the same format
    as we use in the custom message.

POST:
    Submit the URL as is with the body as a JSON string in the format of the custom
    GPS message used in ROS to update the GPS location. We will then publish the
    message to ROS on the topic 'gyroscope'
'''

@app.route('/webhook/gyroscope', methods=['GET', 'POST'])
def gyroscope():
    global gyroscope_data
    if request.method == 'POST':
        json_data = json.loads(request.data)
        gyroscope_data = json_data
        ros.loginfo('/webhook/gyroscope{' + 'x: ' + str(json_data['x']) + ', y: ' + str(json_data['y']) + ', z: ' + str(json_data['z']) + '}')
        gyroscope_pub.send(ros.json_to_msg(request.data, msgs.gyroscope))
        return True
    else:
        return jsonify(gyroscope_data)


'''
Compass update and get webhook
GET:
    If you want to submit a GET method, you can submit the URL as is. The GET method
    will return the last known GPS location as a JSON string using the same format 
    as we use in the custom message.

POST:
    Submit the URL as is with the body as a JSON string in the format of the custom 
    GPS message used in ROS to update the GPS location. We will then publish the 
    message to ROS on the topic 'compass'
'''
@app.route('/webhook/compass', methods=['GET', 'POST'])
def compass():
    global compass_data
    if request.method == 'POST':
        json_data = json.loads(request.data)
        compass_data = json_data
        ros.loginfo('/webhook/compass{' + 'heading: ' + str(json_data['heading']) + '}')
        compass_pub.send(ros.json_to_msg(request.data, msgs.compass))
        return True
    else:
        return jsonify(compass_data)


''' 
Ping function, this is for establishing a cconnection to the micro server. Returns a 
JSON string with status as success. Only GET method allowed
'''
@app.route('/ping')
def ping():
    ros.loginfo("Pinged")
    return True


'''
This initializes the ROS node and sets up publishers
'''
def start_ros():
    global gps_pub, imu_pub, gyroscope_pub, compass_pub
    ros.init_node('micro_server')
    gps_pub = ros.Publisher('sensor_gps_raw', msgs.gps)
    imu_pub = ros.Publisher('sensor_imu_raw', msgs.imu)
    gyroscope_pub = ros.Publisher('sensor_gyroscope_raw', msgs.gyroscope)
    compass_pub = ros.Publisher('sensor_compass_raw', msgs.compass)



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
