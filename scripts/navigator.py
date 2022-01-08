#!/usr/bin/env python
from logging import info
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from direct_sherlock import *


class AutoNavigator:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        self.scan_sub = rospy.Subscriber('scan',LaserScan,self.scan_callback)
        self.dectect_marker = rospy.Subscriber('/turtlebot3_burger/camera/image_raw', Image, self.detect_callback)
        self.node = rospy.init_node('navigator')
        self.command = Twist()
        self.command.linear.x = 0
        self.command.angular.z = 0
        self.rate = rospy.Rate(10)
        self.near_wall = False
        self.min_front = 1
        self.min_right = 1
        self.min_left = 1
        self.min_range = 1
        self.direction = "left"

    def scan_callback(self, msg):
        allranges = msg.ranges
        frontal = allranges[0:5] + allranges[-1:-5:-1]
        rightside = allranges[300:345]
        leftside = allranges[15:60]
        self.min_left = min(leftside)
        self.min_right = min(rightside)
        self.min_front = min(frontal)
        self.min_range = min(self.min_left,self.min_front,self.min_right)

    def detect_callback(self, img):
        pass
        #####################################
        ## Here, you have to detect ArUcos and 
        ## mark the ArUco with notations, 
        ## similar to the one done in week and then display it.
        ## Line joining centre and top_centre must be displayed.
        ## Input: Image data detected by camera sensor
        ## Output : Display marked ArUcos 
        ####################################                             
        
        # img, self.turn = mark_ArUco(cv_image,Detected_ArUco_markers)  

        # if self.turn == "left":
        #     self.direction = "left"
        # elif self.turn == "right":
        #     self.direction = "right"


    def run(self):
        while not rospy.is_shutdown():
            pass
            ############################
            ##If ArUco shows left deirection 
            ## follow right hand side wall. 
            ## If ArUco shows right direction 
            ## follow left hand side wall.

            ## IMP 
            ## You may turn robot to right side even, 
            ## if ArUco is detecting left direction and viceversa.
            ##############################
            # if self.direction == "right":
            #     ######
            # else:
            #     ######

    def left_wall_follow(self):
        ###############################
        ## Follow left hand side wall of robot. 
        ## You can take help from navigator.py file of week3.
        ## You may need to change various values 
        ## form that file to get correct results.
        #################################
        print("follow left walll")

   
    def right_wall_follow(self):
        ###############################
        ## Follow right hand side wall of robot. 
        ## You can take help from navigator.py file of week3.
        ## You may need to change various values 
        ## form that file to get correct results.
        #################################
        print("follow right wall")


if __name__=='__main__':
    navigator = AutoNavigator()
    navigator.run()