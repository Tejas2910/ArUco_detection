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
        # This message has indices based on the angle relative to front of robot
        allranges = msg.ranges
        frontal = allranges[0:5] + allranges[-1:-5:-1]
        rightside = allranges[300:345]
        leftside = allranges[15:60]
        self.min_left = min(leftside)
        self.min_right = min(rightside)
        self.min_front = min(frontal)
        self.min_range = min(self.min_left,self.min_front,self.min_right)

    def detect_callback(self, img):
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        Detected_ArUco_markers = detect_ArUco(cv_image)	                               
        img, self.turn = mark_ArUco(cv_image,Detected_ArUco_markers)  
        #self.turn = give_direction(Detected_ArUco_markers)
        if self.turn == "left":
            self.direction = "left"
        elif self.turn == "right":
            self.direction = "right"
        print(self.direction)
        cv2.namedWindow("Image Window", 1)
        cv2.imshow("Image Window", img)
        k = cv2.waitKey(1)

    def run(self):
        while not rospy.is_shutdown():
            if self.direction == "right":
                self.left_wall_follow()
            else:
                self.right_wall_follow()

    def left_wall_follow(self):
        print("follow left walll")
        if self.near_wall==False and not rospy.is_shutdown():
            print("Moving to wall")
            if self.min_range > 0.2:
                self.command.angular.z = 0.0
                self.command.linear.x = 0.15
            elif self.min_left < 0.2:
                self.near_wall = True
            else:
                self.command.angular.z = -0.20
                self.command.linear.x = 0
            self.cmd_vel_pub.publish(self.command)
            self.rate.sleep()
        else:
            if (self.min_front > 0.2):
                if (self.min_left < 0.12):
                    print("Range: {:.2f}m - Too close. Backing up.".format(self.min_left))
                    self.command.angular.z = -1.2
                    self.command.linear.x = 0.0
                elif self.min_left > 0.15:
                    print("Range: {:.2f}m - Wall-following; turn left.".format(self.min_left))
                    self.command.angular.z = 1.0
                    self.command.linear.x = 0.15
                else:
                    print("Range: {:.2f}m - Wall-following; turn right.".format(self.min_left))
                    self.command.angular.z = -0.1
                    self.command.linear.x = 0.15
            else:
                print("Front obstacle detected. Turning away.")
                self.command.angular.z = -2.0
                self.command.linear.x = -0.02
                self.cmd_vel_pub.publish(self.command)
            self.cmd_vel_pub.publish(self.command)
        self.rate.sleep()
   
    def right_wall_follow(self):
        print("follow right wall")
        if self.near_wall==False and not rospy.is_shutdown():
            print("Moving to wall")
            if self.min_range > 0.2:
                self.command.angular.z = 0.0
                self.command.linear.x = 0.15
            elif self.min_right < 0.2:
                self.near_wall = True
            else:
                self.command.angular.z = 0.20
                self.command.linear.x = 0
            self.cmd_vel_pub.publish(self.command)
            self.rate.sleep()
        else:
            if (self.min_front > 0.2):
                if (self.min_right < 0.12):
                    print("Range: {:.2f}m - Too close. Backing up.".format(self.min_right))
                    self.command.angular.z = 1.2
                    self.command.linear.x = 0.0
                elif self.min_right > 0.15:
                    print("Range: {:.2f}m - Wall-following; turn right.".format(self.min_right))
                    self.command.angular.z = -1.0
                    self.command.linear.x = 0.15
                else:
                    print("Range: {:.2f}m - Wall-following; turn left.".format(self.min_right))
                    self.command.angular.z = 0.1
                    self.command.linear.x = 0.15
            else:
                print("Front obstacle detected. Turning away.")
                self.command.angular.z = 2.0
                self.command.linear.x = -0.02
                self.cmd_vel_pub.publish(self.command)
            self.cmd_vel_pub.publish(self.command)
        self.rate.sleep()


if __name__=='__main__':
    navigator = AutoNavigator()
    navigator.run()