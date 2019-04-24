<<<<<<< HEAD
=======
#/usr/bin/env python
>>>>>>> eb14c64497a1e87aca2d33d4ee6cdc4dfe6a8af1
#!/usr/bin/env python
import rospy
# import cv2
import numpy as np
# import math
# import ros_api as ros
from ros_api import println, is_running
from std_msgs.msg import Float64

import RPi.GPIO as GPIO
from time import sleep
import select
import sys
<<<<<<< HEAD

class H_Bridge():
=======
import keyboard

class H_Bridge(object):
>>>>>>> eb14c64497a1e87aca2d33d4ee6cdc4dfe6a8af1
    def __init__(self, pwm_pin, dir_pin, en_pin, speed=0, direction=0, enabled=True):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.en_pin = en_pin
        self.enabled = enabled
        self.pwm = None
        self.speed = speed
        self.direction = direction
        self.speed = 10
        self.command = " "


<<<<<<< HEAD
=======

>>>>>>> eb14c64497a1e87aca2d33d4ee6cdc4dfe6a8af1
    def init(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.en_pin, GPIO.OUT)

        self.set_dir(self.direction)
        GPIO.output(self.en_pin, GPIO.HIGH)
        self.pwm = GPIO.PWM(self.pwm_pin, 10000)
        self.pwm.start(self.speed)


    def set_speed(self, speed):
<<<<<<< HEAD
        self.speed = speed*2
=======
        self.speed = speed
>>>>>>> eb14c64497a1e87aca2d33d4ee6cdc4dfe6a8af1
        self.pwm.ChangeDutyCycle(self.speed)


    def enable(self):
        self.enebled = True
        GPIO.output(self.en_pin, GPIO.HIGH)

    def disable(self):
        self.enabled = False
        GPIO.output(self.en_pin, GPIO.LOW)

    def set_dir(self, direction):
        self.direction = direction
        if direction == 0:
            GPIO.output(self.dir_pin, GPIO.HIGH)
        else:
            GPIO.output(self.dir_pin, GPIO.LOW)

class DriveController(object):
    def __init__(self,right_wheel,left_wheel):
        self.right = right_wheel
        self.left = left_wheel
        self.left.init()
        self.right.init()
        self.left.set_speed(10)
        self.right.set_speed(10)
        self.right.disable()
        self.left.disable()
        self.des_heading = 0
        self.alpha = 0.25
        self.MIN_OFFSET_THRESH = 15
<<<<<<< HEAD
        self.BIG_OFFSET = 60
        self.ready=False

        rospy.init_node('DriveControl')
=======
        self.BIG_OFFSET = 40

        rospy.init_node('Drive Control Node')
>>>>>>> eb14c64497a1e87aca2d33d4ee6cdc4dfe6a8af1
        hz = 30
        self.rate = rospy.Rate(hz)
        self.heading_sub = rospy.Subscriber('/desired_heading', Float64, self.headingCallback)
        self.des_heading = 0
        # self.ready = False #TODO: Implement some kind of check that the camera is workin
        self.msgread = {'heading': False}

    def headingCallback(self, msg):
<<<<<<< HEAD
=======
        try:
>>>>>>> eb14c64497a1e87aca2d33d4ee6cdc4dfe6a8af1
            self.ready = True
            self.msgread['heading'] = True
            self.des_heading = (1-self.alpha)*self.des_heading + (self.alpha)*msg.data

    def drive_forward(self):
            right.set_dir(0)
            left.set_dir(0)
            right.enable()
            left.enable()
            self.left.set_speed(15)
            self.right.set_speed(15)
<<<<<<< HEAD
            rospy.loginfo('drive forward')
=======
            rospy.loginf('drive forward')
>>>>>>> eb14c64497a1e87aca2d33d4ee6cdc4dfe6a8af1

    def big_right_turn(self):
            right.set_dir(0)
            left.set_dir(0)
            right.enable()
            left.enable()
<<<<<<< HEAD
            self.left.set_speed(35)
            self.right.set_speed(0)
            rospy.loginfo('big right turn')
=======
            self.left.set_speed(20)
            self.right.set_speed(10)
            rospy.loginf('big right turn')
>>>>>>> eb14c64497a1e87aca2d33d4ee6cdc4dfe6a8af1

    def little_right_turn(self):
            right.set_dir(0)
            left.set_dir(0)
            right.enable()
            left.enable()
<<<<<<< HEAD
            self.left.set_speed(25)
            self.right.set_speed(2)
            rospy.loginfo('little right turn')
=======
            self.left.set_speed(15)
            self.right.set_speed(10)
            rospy.loginf('little right turn')
>>>>>>> eb14c64497a1e87aca2d33d4ee6cdc4dfe6a8af1

    def big_left_turn(self):
            right.set_dir(0)
            left.set_dir(0)
            right.enable()
            left.enable()
<<<<<<< HEAD
            self.left.set_speed(0)
            self.right.set_speed(35)
            rospy.loginfo('big left turn')
=======
            self.left.set_speed(10)
            self.right.set_speed(20)
            rospy.loginf('big left turn')
>>>>>>> eb14c64497a1e87aca2d33d4ee6cdc4dfe6a8af1

    def little_left_turn(self):
            right.set_dir(0)
            left.set_dir(0)
            right.enable()
            left.enable()
<<<<<<< HEAD
            self.left.set_speed(2)
            self.right.set_speed(25)
            rospy.loginfo('little left turn')
=======
            self.left.set_speed(10)
            self.right.set_speed(10)
            rospy.loginf('little left turn')
>>>>>>> eb14c64497a1e87aca2d33d4ee6cdc4dfe6a8af1


    def execute(self):
        if self.ready:
            if(abs(self.des_heading) < self.MIN_OFFSET_THRESH):
                self.drive_forward()

            elif(self.des_heading > self.BIG_OFFSET):
                self.big_right_turn()

            elif(self.des_heading > self.MIN_OFFSET_THRESH):
                self.little_right_turn()

            elif(self.des_heading < -self.BIG_OFFSET):
                self.big_left_turn()

            elif(self.des_heading < -self.MIN_OFFSET_THRESH):
                self.little_left_turn()



if __name__ == '__main__':
    println('Starting Drive Control Node')
    left = H_Bridge(17, 22, 27)
    right = H_Bridge(18, 23, 24)
    DriveController = DriveController(right,left)


    while is_running():
<<<<<<< HEAD
        rospy.sleep(1)
=======
        rospy.slepe(1)
>>>>>>> eb14c64497a1e87aca2d33d4ee6cdc4dfe6a8af1
        DriveController.execute()
    GPIO.cleanup()
    println('Node finished with no errors')
