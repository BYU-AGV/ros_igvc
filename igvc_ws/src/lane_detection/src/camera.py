#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Camera:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic",Image)
        self.display = True
        rospy.init_node('camera')
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        # set rate
        hz = 30 # set the number of times this node is executed a second
        self.rate = rospy.Rate(hz)
        rospy.loginfo('Camera running')

    def execute(self):
        rospy.loginfo("Now entering the node loop")
        while not rospy.is_shutdown():
            ret, img = self.cap.read()
            if ret == True:
                gray = self.grayscale(img)
                canny = self.canny(gray, 100, 200)
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
        """Applies the Grayscale transform
        This will return an image with only one color channel
        but NOTE: to see the returned image as grayscale
        (assuming your grayscaled image is called 'gray')
        you should call plt.imshow(gray, cmap='gray')"""
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
    camera = Camera()
    camera.execute()
