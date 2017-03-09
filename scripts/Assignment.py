#!/usr/bin/env python

import rospy
import cv2 
import numpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist 

class assignment:

    def __init__(self):
        cv2.namedWindow("Image window", 1)
        cv2.namedWindow("Thresh", 1)
        self.bridge = CvBridge()
        cv2.startWindowThread()
        self.laser_sub = rospy.Subscriber("/turtlebot/scan/", LaserScan, self.update_laser_data)
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw",Image,self.callback)
        self.motion_pub = rospy.Publisher("/turtlebot/cmd_vel", Twist)
        self.laser_data = []
        self.foundRed = False
        self.foundYellow = False
        self.foundBlue = False
        self.foundGreen = False

    def update_laser_data(self, data):
        self.laser_data = data
    
    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # define range of green color in HSV
        green_min = numpy.array([50,150,50])
        green_max = numpy.array([100,255,255])
    
        blue_min = numpy.array([100,150,50])
        blue_max = numpy.array([150,255,255])
        
        red_min = numpy.array([0,200,100])
        red_max = numpy.array([5,255,255])
        
        yellow_min = numpy.array([20,200,100])
        yellow_max = numpy.array([50,255,195])
        
        h, w, c = hsv_img.shape
        
        # Threshold the HSV image to get only pole colors
        mask_img = numpy.zeros((h, w, 1), dtype="uint8")
        
        if(not self.foundGreen):
            mask_img = cv2.inRange(hsv_img, green_min, green_max)
        
        if(not self.foundBlue):        
            mask_img = cv2.bitwise_or(cv2.inRange(hsv_img, blue_min, blue_max), mask_img)

        if(not self.foundRed):           
            mask_img = cv2.bitwise_or(cv2.inRange(hsv_img, red_min, red_max), mask_img)
            
        if(not self.foundYellow):   
            mask_img = cv2.bitwise_or(cv2.inRange(hsv_img, yellow_min, yellow_max), mask_img)
        
        # Set proportion of image to 0
        top_cutoff = h/4
        bottom_cutoff = h - (h/4)
        #print(h)
        #print(top_cutoff)
        #print(bottom_cutoff)
        
        mask_img[0:top_cutoff, 0:w] = 0
        mask_img[bottom_cutoff:h, 0:w] = 0
        
        base_cmd = Twist()
        base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
        
        
        ranges = numpy.array(self.laser_data.ranges, dtype=numpy.float)
        ranges[numpy.isnan(ranges)] = 99
        midpoint = int(round(len(ranges) / 2))        
        center_range = ranges[midpoint]
        left_ranges = ranges[midpoint-100:midpoint]
        right_ranges = ranges[midpoint:midpoint+100]
        
        if(mask_img.sum() >= 1):
            M = cv2.moments(mask_img)

            if M['m00'] > 0:
              cx = int(M['m10']/M['m00'])
              cy = int(M['m01']/M['m00'])
              
            if(center_range > 0.8):
                err = cx - w/2
                base_cmd.linear.x = 1
                base_cmd.angular.z = (-float(err) / 100)
                    
                self.motion_pub.publish(base_cmd)
            else:
                sub_mat = hsv_img[int(round(w/2))-100:int(round(w/2))+100,int(round(h/2))-100:int(round(h/2))+100]
                
                if(cv2.inRange(sub_mat, green_min, green_max).sum() > 1):
                    self.foundGreen = True
                elif(cv2.inRange(sub_mat, blue_min, blue_max).sum() > 1):
                    self.foundBlue = True
                elif(cv2.inRange(sub_mat, red_min, red_max).sum() > 1):
                    self.foundRed = True
                elif(cv2.inRange(sub_mat, yellow_min, yellow_max).sum() > 1):
                    self.foundYellow = True
                    
        else:
            d = 1.5
            s = 1
            
            if min(left_ranges) > d and min(right_ranges) > d:
                base_cmd.linear.x = s
            elif min(left_ranges) - min(right_ranges) < 0 - d:
                base_cmd.angular.z = s
            elif min(left_ranges) - min(right_ranges) > 0 + d:
                base_cmd.angular.z = -s
            else:
                base_cmd.angular.z = -s
                
            self.motion_pub.publish(base_cmd)
            
        self.motion_pub.publish(base_cmd)
        print self.foundRed,self.foundYellow,self.foundBlue,self.foundGreen, "\n"
        # Bitwise-AND mask and original image
        original_masked = cv2.bitwise_and(cv_image, cv_image, mask=mask_img)
        cv2.imshow("Image window", cv_image)
        cv2.imshow("Thresh", original_masked)

rospy.init_node('assignment')
tf = assignment()

rospy.spin()

cv2.destroyAllWindows()