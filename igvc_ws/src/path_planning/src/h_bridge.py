#/usr/bin/env python

import RPi.GPIO as GPIO
from time import sleep
import select
import sys
import keyboard


#def setup():
#    GPIO.setmode(GPIO.BCM)
#    GPIO.setup(17, GPIO.OUT)
#    GPIO.setup(18, GPIO.OUT)
#    GPIO.setup(22, GPIO.OUT)
#    GPIO.setup(23, GPIO.OUT)
#
#    GPIO.output(17, GPIO.HIGH)
#    GPIO.output(18, GPIO.HIGH)
#    GPIO.output(22, GPIO.LOW)
#    GPIO.output(23, GPIO.LOW)
#
#
#def run():
#    try:
#        while True:
#            GPIO.output(17, GPIO.HIGH)
#            GPIO.output(18, GPIO.HIGH)
#            GPIO.output(22, GPIO.LOW)
#            GPIO.output(23, GPIO.LOW)
#
#            pwm = GPIO.PWM(17, 1000)
#            pwm.start(100)
#            print 'Testing forward'
#            GPIO.output(23, GPIO.HIGH)
#            for dc in range(100, 0, -1):
#                pwm.ChangeDutyCycle(dc)
#                sleep(0.01)
#
#            for dc in range(0, 100):
#                pwm.ChangeDutyCycle(dc)
#                sleep(0.01)
#
#            pwm.stop()
#            GPIO.output(23, GPIO.LOW)
#            GPIO.output(17, GPIO.HIGH)
#            sleep(2)
#
#            GPIO.output(17, GPIO.HIGH)
#            GPIO.output(18, GPIO.HIGH)
#            GPIO.output(22, GPIO.LOW)
#            GPIO.output(23, GPIO.LOW)
#
#            print 'Testing reverse'
#            pwm = GPIO.PWM(18, 1000)
#            pwm.start(100)
#            GPIO.output(22, GPIO.HIGH)
#            for dc in range(100, 0, -1):
#                pwm.ChangeDutyCycle(dc)
#                sleep(0.01)
#            for dc in range(0, 100):
#                pwm.ChangeDutyCycle(dc)
#                sleep(0.01)
#            pwm.stop()
#            GPIO.output(22, GPIO.LOW)
#            GPIO.output(18, GPIO.HIGH)
#            sleep(2)
#
#
#    except KeyboardInterrupt:
#        pass
#
#
#
#def cleanup():
#    print 'Cleaning up'
#    GPIO.cleanup()
#
#
#
#class H_Bridge(object):
#    def __init__(self, left_pin, right_pin, left_pulse, right_pulse):
#        self.l_pin = left_pin
#        self.r_pin = right_pin
#        self.l_pulse = left_pulse
#        self.r_pulse = right_pulse
#        self.pwm_l = None
#        self.pwm_r = None
#        self.state = 'stop'
#
#    def setup(self):
#        GPIO.setmode(GPIO.BCM)
#        # Setup pins so that GPIO knows which pins do what
#        GPIO.setup(l_pin, GPIO.OUT)
#        GPIO.setup(r_pin, GPIO.OUT)
#        GPIO.setup(l_pulse, GPIO.OUT)
#        GPIO.setup(r_pulse, GPIO.OUT)
#        # Set initial values for the pins
#        GPIO.output(l_pin, GPIO.LOW)
#        GPIO.output(r_pin, GPIO.LOW)
#        GPIO.output(l_pulse, GPIO.HIGH)
#        GPIO.output(r_pulse, GPIO.HIGH)
#
#    
#    def clip(self, speed):
#        if speed > 100:
#            return 100
#        if speed < 0:
#            return 0
#        return speed
#
#
#    def stop(self):
#        if self.pwm_l != None:
#            self.pwm_l.stop()
#            self.pwm_l = None
#        if self.pwm_r != None:
#            self.pwm_r.stop()
#            self.pwm_r = None
#
#        GPIO.output(self.l_pin, GPIO.LOW)
#        GPIO.output(self.r_pin, GPIO.LOW)
#        GPIO.output(self.l_pulse, GPIO.HIGH)
#        GPIO.output(self.r_pulse, GPIO.HIGH)
#        sleep(0.1)
#
#        self.state = 'stop'
#        print 'Stopping'
#        print self
#
#
#    def forward(self, speed):
#        #print ''
#        #print 'FORWARD'
#        #print self.state
#        #print self.pwm_l
#        speed = self.clip(speed)
#
#        #if self.state != 'forward' or self.state != 'stop':
#        if self.state != 'forward':
#            self.stop()
#            self.state = 'forward'
#
#        if self.pwm_l == None:
#            #print 'Starting to forward'
#            self.pwm_l = GPIO.PWM(self.l_pulse, 1000)
#            self.pwm_l.start(100 - speed)
#        else:
#            self.pwm_l.ChangeDutyCycle(100 - speed)
#        GPIO.output(self.r_pin, GPIO.HIGH)
#
#
#    def reverse(self, speed):
#        #print ''
#        #print 'REVERSE'
#        #print self.state
#        #print self.pwm_r
#        speed = self.clip(speed)
#
#        #if self.state != 'reverse' or self.state != 'stop':
#        if self.state != 'reverse':
#            self.stop()
#            self.state = 'reverse'
#
#        if self.pwm_r == None:
#            #print 'Starting to reverse'
#            self.pwm_r = GPIO.PWM(self.r_pulse, 1000)
#            self.pwm_r.start(100 - speed)
#        else:
#            self.pwm_r.ChangeDutyCycle(100 - speed)
#        GPIO.output(self.l_pin, GPIO.HIGH)
#
#
#if __name__ == '__main__':
#    setup()
#    print 'Creating new H-Bridge Controller'
#    hb = H_Bridge(17, 18, 22, 23)
#    hb.stop()
#    print hb
#    hb.reverse(100)
#    sleep(1)
#    hb.stop()
#    print 'Testing forward 50'
#    hb.forward(50)
#    sleep(1)
#    print 'Testing forward 100'
#    hb.forward(100)
#    sleep(1)
#    print 'Testing forward 0'
#    hb.forward(0)
#    sleep(1)
#    print 'Testing stop from forward 100'
#    hb.forward(100)
#    sleep(1)
#    hb.stop()
#    sleep(1)
#
#    print 'Testing reverse 50'
#    hb.reverse(50)
#    sleep(1)
#    print 'Testing reverse 100'
#    hb.reverse(100)
#    sleep(1)
#    print 'Testing reverse 0'
#    hb.reverse(0)
#    sleep(1)
#    print 'Testing stop from reverse'
#    hb.reverse(100)
#    sleep(1)
#    hb.stop()
#    sleep(1)
#
#    print 'Testing forward from reverse'
#    hb.forward(100)
#    sleep(1)
#    hb.stop()
#    hb.reverse(100)
#    sleep(1)
#
#    hb.stop()
#    sleep(2)
#    cleanup()

class H_Bridge(object):
    def __init__(self, pwm_pin, dir_pin, en_pin, speed=0, direction=0, enabled=True):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.en_pin = en_pin
        self.enabled = enabled
        self.pwm = None
        self.speed = speed
        self.direction = direction


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
    left = H_Bridge(17, 22, 27)
    right = H_Bridge(18, 23, 24)
    left.init()
    right.init()
    left.set_speed(10)
    right.set_speed(10)
    right.disable()
    left.disable()
    try :

        while True:
            command = raw_input()
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
                speed = input("Enter speed: ")
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
            pass
        
    GPIO.cleanup()


