#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import custom_msgs.msg as msgs
import ros_api as ros
from ros_api import println, ros_is_running, ros_sleep, ROS_Handler


class LaneDetector(ROS_Handler):
    def __init__(self):
        ROS_Handler.__init__(self,'lane_detection', 'camera_feed', Image, 30)
        self.image_sub = rospy.Subscriber('camera_feed', Image, self.imageCallback)
        self.display = rospy.get_param('~lane_display')
        self.ready = False #TODO: Implement some kind of check that the camera is workin
        self.img = 0
        self.msgread = {'image': False}
        self.bridge = CvBridge()
        println('Lane detection running')

    def execute(self):
        while ros_is_running():
            img = self.img
            # println(np.max(img),np.min(img))
            if self.ready and self.display:
                gray = self.grayscale(img)
                canny = self.canny(gray, 50, 150)
                if self.display:
                    cv2.imshow("input", canny)
                else:
                    pass
            else:
                pass
            key = cv2.waitKey(10)
            if key == 27:
                break
    def grayscale(self,img):
        result = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return result

    def canny(self, img, low_threshold, high_threshold):
        """Applies the Canny transform"""
        return cv2.Canny(img, low_threshold, high_threshold)

    def gaussian_blur(self,img, kernel_size):
        """Applies a Gaussian Noise kernel"""
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
        # Or use BGR2GRAY if you read an image with cv2.imread()
        # return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    def imageCallback(self, msg):
        try:
            self.ready = True
            self.img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            self.msgread['image'] = True
        except CvBridgeError as e:
            print(e)
if __name__ == '__main__':
    println('Starting lane detection node')
    lane_detector = LaneDetector()
    while ros_is_running():
        ros_sleep(1)
        lane_detector.execute()

    println('Node finished with no errors')
