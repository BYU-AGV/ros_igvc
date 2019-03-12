#!/usr/bin/env python
import rospy
import cv2
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import custom_msgs.msg as msgs
import ros_api as ros
from ros_api import println, is_running

class Camera():
    def __init__(self):
        # Handler.__init__(self,'camera', 'camera_feed', Image, 30)
        rospy.init_node('camera_feed')
        self.hz = 30
        self.rate = rospy.Rate(self.hz)
        self.image_pub = rospy.Publisher('camera_feed', Image, queue_size=1)
        self.display = rospy.get_param('~cam_display')
        self.cap = cv2.VideoCapture(0)
        if (self.cap.isOpened()== False):
            rospy.logwarn('CAMERA NOT OPEN')
        self.bridge = CvBridge()
        # rospy.loginfo('Camera running')
        println('Camera running')

    def execute(self):
        rospy.loginfo("Now entering the node loop")
        while is_running():
            ret, img = self.cap.read()
            if ret == True:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
                if self.display:
                    cv2.imshow("input", img)
                else:
                    pass
            else:
                pass
            key = cv2.waitKey(10)
            if key == 27:
                break
        # Or use BGR2GRAY if you read an image with cv2.imread()
        # return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

if __name__ == '__main__':
    println('Starting camera node')
    camera = Camera()
    while is_running():
        camera.execute()
        self.rate.sleep()
    println('Node finished with no errors')
