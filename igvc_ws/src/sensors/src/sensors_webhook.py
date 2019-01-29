#!/usr/bin/env python

import socket
import fcntl
import struct
import os

import rospy
from custom_msgs.msg import *
from flask import Flask, request, abort, Response, jsonify

app = Flask(__name__)


@app.route('/webhook')
def webhook():
    return "Webhooks :)"


@app.route('/accel', methods=['POST'])
def accel():
    return "success"


@app.route('/ping')
def ping():
    rospy.loginfo('/ping')
    return jsonify({'status': 'success'})


def startROS():
    pub = rospy.Publisher('sensor_webhook', coord, queue_size=10)
    rospy.init_node('sesnor_webhook_server', anonymous=True)
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
    app.run(host='0.0.0.0')
