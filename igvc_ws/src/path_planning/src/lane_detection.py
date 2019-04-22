#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import custom_msgs.msg as msgs
from std_msgs.msg import Float64
import ros_api as ros
from ros_api import println, is_running
import pathfinding


class LaneDetector():
    def __init__(self):
        rospy.init_node('lane_detection')
        hz = 30
        self.rate = rospy.Rate(hz)
        self.image_sub = rospy.Subscriber('/raspicam_node/image', Image, self.imageCallback)
        self.heading_pub = rospy.Publisher('desired_heading', Float64, queue_size=1)
        self.result_pub = rospy.Publisher('/pathfinding_feed', Image, queue_size=1)
        self.display = rospy.get_param('~lane_display')
        self.ready = False #TODO: Implement some kind of check that the camera is workin
        self.img = 0
        self.msgread = {'image': False}
        self.bridge = CvBridge()
        self.num_col = 21
        self.num_row = 10
        self.num_ele = self.num_col*self.num_row
        self.start_pos = [self.num_row-1,(self.num_col-1)/2]
        self.node_thresh = 0.5 #This is the threshold for driveable and not driveable nodes. Is between 0 and 1

        println('Lane detection running')

    def GetDriveableNodes(self, img):
        #img = img[0:640,:,:]
        img = cv2.GaussianBlur(img,(11,11),0)
        sz = img.shape
        num_col = self.num_col
        num_row = self.num_row
        win_width = sz[0]//num_row
        win_height = sz[1]//num_col
        sigma = 200 #Smoothing Term
        index = 0
        nodes = []
        node_scores = np.empty(num_col*num_row, dtype = np.float32)

        hls = cv2.cvtColor(img,cv2.COLOR_RGB2HLS)
        light = hls[:,:,1] > 140
        saturation = hls[:,:,2] < 100
        sat_and_light = np.logical_and(saturation, light)

        for r in range(num_row):
            for c in range(num_col):
                nodes.append(sat_and_light[r*win_width:(r+1)*win_width,c*win_height:(c+1)*win_height])

        #Assign each node a score based on the number of lane pixels we observe
        for node in nodes:
            num_lane_pixels = np.sum(node)
            score = np.exp(-(num_lane_pixels/sigma)**2)
            node_scores[index] = score
            index += 1
        node_scores = np.reshape(node_scores,(num_row,num_col))

        #Visualizing by overlaying on the image...
        overlayed_img = np.zeros_like(img,dtype = np.uint8)
        for r in range(num_row):
            for c in range(num_col):
        #         print(node_scores[r,c])
                overlayed_img[r*win_width:(r+1)*win_width,c*win_height:(c+1)*win_height] = (node_scores[r,c])*255 #This line isn't working like I would have hoped...
        #Combine the original and overlayed_img

        return node_scores, overlayed_img

    def buildAlgorithmStructures(self,node_scores):
        kernel = np.array([[0, 1, 0],
                           [1, 0, 1],
                           [0, 1, 0]])
        nodes = [[0,0]] #Instantiate the first node
        for i in range(self.num_row):
            for j in range(self.num_col):
                if(i == 0 and j ==0):
                    pass
                else:
                    nodes = np.append(nodes,[[i,j]],axis = 0)

        edges = np.zeros([self.num_row,self.num_col])
        edges = np.reshape(edges,(1,self.num_ele))
        for i in range(self.num_row):
            for j in range(self.num_col):
                edge_row = np.zeros([self.num_row,self.num_col])
                if(node_scores[i,j] > self.node_thresh):
                    if(i == 0 and j == 0):
                        edge_row[(i):(i+2),(j):(j+2)] += kernel[0:2,0:2]
                    elif(i == 0 and j == self.num_col-1):
                        edge_row[(i):(i+2),(j-2):(j)] += kernel[1:3,0:2]
                    elif(i == self.num_row-1 and j == 0):
                        edge_row[(i-2):(i),(j):(j+2)] += kernel[0:2,1:3]
                    elif(i == self.num_row-1 and j == self.num_col-1):
                        edge_row[(i-2):(i),(j-2):(j)] += kernel[1:3,1:3]
                    elif(i == 0):
                        edge_row[(i):(i+2),(j-1):(j+2)] += kernel[1:3,0:3]
                    elif(j == 0):
                        edge_row[(i-1):(i+2),(j):(j+2)] += kernel[0:3,1:3]
                    elif(i == self.num_row-1):
                        edge_row[(i-1):(i+1),(j-1):(j+2)] += kernel[0:2,0:3]
                    elif(j == self.num_col-1):
                        edge_row[(i-1):(i+2),(j-1):(j+1)] += kernel[0:3,0:2]
                    else:
                        edge_row[(i-1):(i+2),(j-1):(j+2)] += kernel
                edge_row = np.reshape(edge_row,(1,self.num_ele))
                edges = np.append(edges,edge_row,axis = 0)

        edges = edges[1:self.num_ele+1,:] #Removing the first element we appended for the shape

        return nodes, edges

    def plot_path(self,img, path):
        sz = img.shape
        num_col = self.num_col
        num_row = self.num_row
        win_width = sz[0]//num_row
        win_height = sz[1]//num_col
        orig_img = img
        red = [0, 0, 255]
        for r,c in path:
            r = int(r)
            c = int(c)
            img[r*win_width:(r+1)*win_width,c*win_height:(c+1)*win_height] = red
        return self.weighted_img(orig_img, img)


    def weighted_img(self,img, initial_img, a=0.0, b=1., y=0.):

        return cv2.addWeighted(initial_img, a, img, b, y)

    def imageCallback(self, msg):
        try:
            self.ready = True
            self.img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            self.msgread['image'] = True
        except CvBridgeError as e:
            print(e)

    def piImageCallback(self, msg):
        try:
            self.ready = True
            self.img = self.bridge.imgmsg_to_cv2(msg)
            self.msgread['image'] = True
        except CvBridgeError as e:
            print(e)

    def execute(self):
        while is_running():
            img = self.img
            # println(np.max(img),np.min(img))
            if self.ready and self.display:
                node_scores, processed_img = self.GetDriveableNodes(img)
                list_of_nodes, list_of_edges = self.buildAlgorithmStructures(node_scores)
                end_pos = [0,(self.num_col-1)/2]
                path = pathfinding.search(list_of_nodes.tolist(), list_of_edges.tolist(), self.start_pos, end_pos, 'bfs')
                path_img = self.plot_path(processed_img,path)
                desired_heading = self.get_heading(path)
                self.heading_msg.data = desired_heading
                self.heading_pub.publish(self.heading_msg)
                self.result_pub.publish(self.bridge.cv2_to_imgmsg(path_img, "bgr8"))
            else:
                pass
            key = cv2.waitKey(10)
            if key == 27:
                break

    def get_heading(self, path, num_blocks = 5):
        o_r, o_c = path[0]
        heading = 0 #Straight ahead is zero, to the right is positive and to the left is negative
        dx = 0
        dy = 0
        if(len(path) < num_blocks):
            num_blocks = len(path)
        for r,c in path[1:num_blocks]:
            # r = int(r)
            # c = int(c)
            dx += (r - o_r)
            dy += (c - o_c)
        if dx == 0 and dy > 0:
            return 90
        elif dx == 0 and dy < 0:
            return -90
        elif dx == 0 and dy == 0:
            return 0
        else:
            heading =  -math.degrees(math.atan(dy/dx))
            return heading

            img[r*win_width:(r+1)*win_width,c*win_height:(c+1)*win_height] = red




if __name__ == '__main__':
    println('Starting lane detection node')
    lane_detector = LaneDetector()
    while is_running():
        rospy.sleep(1)
        lane_detector.execute()

    println('Node finished with no errors')
