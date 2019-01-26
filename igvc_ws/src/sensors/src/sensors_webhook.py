#!/usr/bin/env python

import rospy
from custom_msgs.msg import *
from flask import Flask, request, abort, Response, jsonify

app = Flask(__name__)


@app.route('/webhook')
def webhook():
    return "Webhooks :)"


@app.route('/ping')
def ping():
    rospy.loginfo('/ping')
    return jsonify({'status': 'success'})


def startROS():
    pub = rospy.Publisher('sensor_webhook', coord, queue_size=10)
    rospy.init_node('sesnor_webhook_server', anonymous=True)
    rospy.loginfo("Webhook is starting")



if __name__ == '__main__':
    startROS()
    app.run(host='192.168.226.132')
