#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import custom_msgs.msg as msgs
import ros_api as ros
from ros_api import println, ros_is_running, ros_sleep, ROS_Handler

class Camera(ROS_Handler):
    def __init__(self):
        ROS_Handler.__init__(self,'camera', 'camera_feed', Image, 30)
        self.image_pub = rospy.Publisher('camera_feed', Image, queue_size=1)
        self.display = rospy.get_param('~cam_display')
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        # rospy.loginfo('Camera running')
        println('Camera running')

    def execute(self):
        rospy.loginfo("Now entering the node loop")
        while ros_is_running():
            ret, img = self.cap.read()
            if ret == True:
                # gray = self.grayscale(img)
                # canny = self.canny(gray, 100, 200)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
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
        result = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        return result

    def canny(self, img, low_threshold, high_threshold):
        """Applies the Canny transform"""
        return cv2.Canny(img, low_threshold, high_threshold)

    def gaussian_blur(self,img, kernel_size):
        """Applies a Gaussian Noise kernel"""
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
        # Or use BGR2GRAY if you read an image with cv2.imread()
        # return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

if __name__ == '__main__':
    println('Starting camera node')
    camera = Camera()
    while ros_is_running():
        camera.execute()
        ros_sleep(1)
    println('Node finished with no errors')
