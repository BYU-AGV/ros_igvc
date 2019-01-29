#!/usr/bin/env python

import socket
import fcntl
import struct
import os
import json as j

import rospy
import ros_api as ros

import custom_msgs.msg as msg
import std_msgs as std_msgs

from flask import Flask, request, abort, Response, jsonify

app = Flask(__name__)

# Varialbes go here
publisher = None # this is the publisher
gps_location = None
acceleration = None

@app.route('/webhook')
def webhook():
    return "Webhooks :)"


@app.route('/webhook/accelerometer', methods=['GET', 'POST'])
def accel():
    global acceleration
    if request.method == 'POST':
        data = request.data
        json = j.loads(data)
        acceleration = json
        rospy.loginfo('/webhook/accelerometer{' +  'type: ' + str(json['type']) + ', x: ' + str(json['x']) + ', y: ' + str(json['y']) + ', z: ' + str(json['z']) + '}')
        return 'success'
    elif request.method == 'GET':
        #rospy.loginfo("Getting gps location")
        return jsonify(acceleration)


@app.route('/webhook/gps', methods=['GET', 'POST'])
def gps():
    global gps_location
    if request.method == 'POST':
        json = j.loads(request.data)
        gps_location = json
        rospy.loginfo('/webhook/gps{' + 'latitude: ' +  str(json['latitude']) + ', longitude: ' + str(json['longitude']) + ', altitude: ' + str(json['altitude']) + ', accuracy: ' + str(json['accuracy']) + ' speed: ' + str(json['speed']) + ', speed_accuracy: ' +  str(json['speed_accuracy']) + '}')
        
        if publisher is not None:
            publisher.send(ros.json_to_msg(request.data, msg.gps))

        return 'success'
    else:
        return jsonify(gps_location)
    return 'failed'



@app.route('/ping')
def ping():
    #rospy.loginfo('/ping')
    return jsonify({'status': 'success'})


def startROS():
    #publisher = rospy.Publisher('sensor_webhook', msg.accelerometer, queue_size=10)
    #rospy.init_node('sesnor_webhook_server', anonymous=True)
    global publisher
    publisher = ros.ROS_Publisher('gps_publisher', 'gps_location', msg.gps)
    rospy.loginfo("Webhook is starting")



def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])


def get_inet_address():
    f = os.popen('ifconfig ens33 | grep "inet [0-9]\{1,3\}\.[0-9]\{1,3\}\.[0-9]\{1,3\}\.[0-9]\{1,3\}"')
    return f.read()


if __name__ == '__main__':
    print "Running on: " + get_inet_address()
    startROS()
    print("Starting the server")
    app.run(host='0.0.0.0')
