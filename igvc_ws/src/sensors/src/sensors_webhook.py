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
    return jsonify({'status': 'success'})


if __name__ == '__main__':
    app.run(host='192.168.226.132')
