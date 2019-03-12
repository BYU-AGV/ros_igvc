#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import custom_msgs.msg as msgs
import ros_api as ros
from ros_api import println, is_running
import pathfinding


class LaneDetector():
    def __init__(self):
        rospy.init_node('lane_detection')
        hz = 30
        self.rate = rospy.Rate(hz)
        self.image_sub = rospy.Subscriber('camera_feed', Image, self.imageCallback)
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
        img = img[0:640,:,:]
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

    def execute(self):
        while is_running():
            img = self.img
            # println(np.max(img),np.min(img))
            if self.ready and self.display:
                node_scores, processed_img = self.GetDriveableNodes(img)
                list_of_nodes, list_of_edges = self.buildAlgorithmStructures(node_scores)
                end_pos = [0,(self.num_col-1)/2]
                path = pathfinding.search(list_of_nodes, list_of_edges, self.start_pos, end_pos, 'A*')
                path_img = self.plot_path(processed_img,path)
                # rospy.loginfo(path)
                if self.display:
                    cv2.imshow("input", path_img)
                else:
                    pass
            else:
                pass
            key = cv2.waitKey(10)
            if key == 27:
                break





if __name__ == '__main__':
    println('Starting lane detection node')
    lane_detector = LaneDetector()
    while is_running():
        rospy.sleep(1)
        lane_detector.execute()

    println('Node finished with no errors')
