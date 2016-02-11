#/usr/bin/env python
#!/usr/bin/env python
#import rospy
# import cv2
import numpy as np
# import math
# import ros_api as ros


import RPi.GPIO as GPIO
from time import sleep
import select
import sys
#import keyboard

class H_Bridge(object):
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
        self.speed = speed
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






if __name__ == '__main__':
#    println('Starting H-Bridge Node')
    left = H_Bridge(17, 22, 27)
    right = H_Bridge(18, 23, 24)
    left.init()
    right.init()
    left.set_speed(10)
    right.set_speed(10)
    right.disable()
    left.disable()
    speed = 20

    try:
        while True:
            command = input()
            command = str(command)
            if command == "stop" or command == " ":
                right.disable()
                left.disable()
            elif command == "forward" or command == "w":
                right.set_dir(0)
                left.set_dir(0)
                right.enable()
                left.enable()

            elif command == "reverse" or command == "s":
                right.set_dir(1)
                left.set_dir(1)
                right.enable()
                left.enable()

            elif command == "end" or command == "e":
                break
            elif command == "speed" or command == "z":
                speed -= 10
                right.set_speed(speed)
                left.set_speed(speed)
            elif command == "speed" or command == "c":
                speed += 10
                right.set_speed(speed)
                left.set_speed(speed)
            elif command == "right" or command == "d":
                right.enable()
                left.enable()
                right.set_dir(1)
                left.set_dir(0)

            elif command == "left" or command == "a":
                right.enable()
                left.enable()
                right.set_dir(0)
                left.set_dir(1)

    except KeyboardInterrupt:
       GPIO.cleanup()
