#!/usr/bin/env python

import rospy
import cv2 
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist 

class turtlebot_follower:

    def __init__(self):

        cv2.namedWindow("Image window", 1)
        cv2.namedWindow("Thresh", 1)
        self.bridge = CvBridge()
        cv2.startWindowThread()
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw",Image,self.callback)
        self.motion_pub = rospy.Publisher("/turtlebot/cmd_vel", Twist)
    
    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # define range of green color in HSV
        green_min = numpy.array([50,150,50])
        green_max = numpy.array([255,255,255])
    
        red_min = numpy.array([0,200,100])
        red_max = numpy.array([5,255,255])
        
        yellow_min = numpy.array([20,200,100])
        yellow_max = numpy.array([50,255,195])
        
        # Threshold the HSV image to get only green colors
        mask_img = cv2.inRange(hsv_img, green_min, green_max)
        mask_img = cv2.bitwise_or(cv2.inRange(hsv_img, red_min, red_max), mask_img)
        mask_img = cv2.bitwise_or(cv2.inRange(hsv_img, yellow_min, yellow_max), mask_img)
        
        # Set proportion of image to 0
        h, w = mask_img.shape
        top_cutoff = h/4
        bottom_cutoff = h - (h/4)
        #print(h)
        #print(top_cutoff)
        #print(bottom_cutoff)
        
        mask_img[0:top_cutoff, 0:w] = 0
        mask_img[bottom_cutoff:h, 0:w] = 0
        
        if(mask_img.sum() >= 1):
            M = cv2.moments(mask_img)

            if M['m00'] > 0:
              cx = int(M['m10']/M['m00'])
              cy = int(M['m01']/M['m00'])
              
            base_cmd = Twist()
            base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;  
            err = cx - w/2
            base_cmd.linear.x = 0.5
            base_cmd.angular.z = -float(err) / 100
            print base_cmd
                
            self.motion_pub.publish(base_cmd)
            
        # Bitwise-AND mask and original image
        original_masked = cv2.bitwise_and(cv_image, cv_image, mask=mask_img)
        cv2.imshow("Image window", cv_image)
        cv2.imshow("Thresh", original_masked)

rospy.init_node('turtlebot_follower')
tf = turtlebot_follower()

rospy.spin()

cv2.destroyAllWindows()