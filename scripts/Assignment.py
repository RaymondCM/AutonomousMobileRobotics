#!/usr/bin/env python

import rospy
import cv2 
import numpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist 
from random import randint

class assignment:
    
    green_min = numpy.array([50,150,50])
    green_max = numpy.array([100,255,255])
    blue_min = numpy.array([100,150,50])
    blue_max = numpy.array([150,255,255])
    red_min = numpy.array([0,200,100])
    red_max = numpy.array([5,255,255])
    yellow_min = numpy.array([20,200,100])
    yellow_max = numpy.array([50,255,195])
    
    wheel_radius = 0.076
    robot_radius = 0.43
    
    h = 0
    w = 0
    
    foundRed = False
    foundYellow = False
    foundBlue = False
    foundGreen = False
    
    laser_data = []
    
    def __init__(self):
        cv2.namedWindow("Image window", 1)
        cv2.namedWindow("Thresh", 1)
        self.bridge = CvBridge()
        cv2.startWindowThread()
        self.laser_sub = rospy.Subscriber("/turtlebot/scan/", LaserScan, self.update_laser_data)
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw",Image,self.callback)
        self.motion_pub = rospy.Publisher("/turtlebot/cmd_vel", Twist)
        
    def update_laser_data(self, data):
        self.laser_data = data.ranges
    
    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # define range of green color in HSV
        self.h, self.w, c = hsv_img.shape
        
        # Threshold the HSV image to get only pole colors
        mask_img = self.threshold_image(hsv_img)
        
        #Remove top and bottom of the image
        mask_img[0:self.h/4, 0:self.w] = 0
        mask_img[self.h - (self.h/4):self.h, 0:self.w] = 0
        
        self.naive_behaviour(hsv_img, mask_img)
        self.results(cv_image, mask_img)
    
    def threshold_image(self, hsv_img):
        mask_img = numpy.zeros((self.h, self.w, 1), dtype="uint8")
        
        if(not self.foundGreen):
            mask_img = cv2.inRange(hsv_img, self.green_min, self.green_max)
        
        if(not self.foundBlue):        
            mask_img = cv2.bitwise_or(cv2.inRange(hsv_img, self.blue_min, self.blue_max), mask_img)

        if(not self.foundRed):           
            mask_img = cv2.bitwise_or(cv2.inRange(hsv_img, self.red_min, self.red_max), mask_img)
            
        if(not self.foundYellow):   
            mask_img = cv2.bitwise_or(cv2.inRange(hsv_img, self.yellow_min, self.yellow_max), mask_img)
        
        return mask_img
        
    def results(self, img, mask):
        print self.foundRed, self.foundYellow, self.foundBlue, self.foundGreen, "\n"
        cv2.imshow("Image window", img)
        cv2.imshow("Thresh", cv2.bitwise_and(img, img, mask=mask))
            
    def f_kinematics(self, w_l, w_r):
        c_l = self.wheel_radius * w_l
        c_r = self.wheel_radius * w_r
        v = (c_l + c_r) / 2
        a = (c_l - c_r) / self.robot_radius
        return (v, a)   
    
    def check_object(self, image):
        sub_mat = image[int(round(self.w/2))-100:int(round(self.w/2))+100,int(round(self.h/2))-100:int(round(self.h/2))+100]
                
        if(cv2.inRange(sub_mat, self.green_min, self.green_max).sum() > 1):
            self.foundGreen = True
            self.found_object()
        elif(cv2.inRange(sub_mat, self.blue_min, self.blue_max).sum() > 1):
            self.foundBlue = True
            self.found_object()
        elif(cv2.inRange(sub_mat, self.red_min, self.red_max).sum() > 1):
            self.foundRed = True
            self.found_object()
        elif(cv2.inRange(sub_mat, self.yellow_min, self.yellow_max).sum() > 1):
            self.foundYellow = True
            self.found_object()
    
    def found_object(self):
        m = Twist()
        m.linear.x = m.linear.y = m.angular.z = 0
        self.motion_pub.publish(m)
        rospy.sleep(2)
        
    def naive_behaviour(self, hsv_img, mask_img):
        
        h, w, c = hsv_img.shape
        base_cmd = Twist()
        base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0
        
        ranges = numpy.array(self.laser_data, dtype=numpy.float)
        
        midpoint = int(round(len(ranges) / 2))  
        lr_len = len(ranges) / 2 #45* of data
        
        left_to_mid = ranges[midpoint-lr_len:midpoint]
        mid_to_right = ranges[midpoint:midpoint+lr_len]
        
        left_range = numpy.nanmin(left_to_mid)
        right_range = numpy.nanmin(mid_to_right)
        center_range = ranges[midpoint]
        
        
        if(mask_img.sum() >= 1):
            M = cv2.moments(mask_img)

            if M['m00'] > 0:
              cx = int(M['m10']/M['m00'])
              #cy = int(M['m01']/M['m00'])
              
            if(center_range > 0.8):
                err = cx - w/2
                base_cmd.linear.x = 1
                base_cmd.angular.z = (-float(err) / 100)
                    
                self.motion_pub.publish(base_cmd)
            else:
                self.check_object(hsv_img)
                    
        else:
            d = 1 # min_distace
            x_s = 1 #x speed
            z_s = 1 #angular speed
            
            direction_min = min([left_range, center_range, right_range])
            if left_range > d and right_range > d and direction_min > 1:
                base_cmd.linear.x = x_s
            elif left_range - right_range < -d:
                base_cmd.angular.z = z_s
            elif left_range - right_range > +d:
                base_cmd.angular.z = -z_s
            else:
                base_cmd.angular.z = z_s
                
            self.motion_pub.publish(base_cmd)
            
        self.motion_pub.publish(base_cmd)
        

rospy.init_node('assignment')
tf = assignment()

rospy.spin()

cv2.destroyAllWindows()