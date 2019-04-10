#!/usr/bin/env python

import rospy

from picamera.array import PiRGBArray
from picamera import PiCamera
#from custom_msgs.msg import image 
import time
import custom_msgs.msg as msgs 
import numpy as np

class pi_cam():
	def __init__(self):
# initialize the camera and grab a reference to the raw camera capture
		self.camera = PiCamera()
		self.rawCapture = PiRGBArray(self.camera)
		self.pub_image = rospy.Publisher('/pi_cam_images', msgs.image, queue_size=1) 
 
# grab an image from the camera

	def execute(self):
		rospy.loginfo('Execute Loop')
		while not rospy.is_shutdown():
			self.camera.capture(self.rawCapture, format="bgr")
			self.image = self.rawCapture.array
                        print(self.image)
                        print(self.image.flatten())
                        a = np.array(self.image.flatten())
			self.pub_image.publish(480, 720, a.tolist())
			self.rawCapture.truncate(0)
			rospy.sleep(1.0)		
if __name__ == "__main__":
    rospy.init_node('pi_camera_node', anonymous=True)
    cam = pi_cam()
    cam.execute()
    
    #    rospy.ROSInterruptException
    #pass

