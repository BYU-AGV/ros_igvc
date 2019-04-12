#!/usr/bin/env python

import rospy
import time
import custom_msgs.msg as msgs 
import numpy as np

class pi_cam_receiver():
        def __init__(self):
# initialize the camera and grab a reference to the raw camera capture
               self.sub_image = rospy.Subscriber('/pi_cam_images', msgs.image, self.imageCallback)
	       self.image = None
	def imageCallback(self, msg):
	       self.image = msg.data
# grab an image from the camera

        def execute(self):
                rospy.loginfo('Execute Loop')
                while not rospy.is_shutdown():
                        if(self.image is not None):
				print(self.image)
                        	rospy.sleep(1.0)
if __name__ == "__main__":
    rospy.init_node('image_receiver', anonymous=True)
    receiver = pi_cam_receiver()
    receiver.execute()
    
    #    rospy.ROSInterruptException
    #pass

