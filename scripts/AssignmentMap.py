#!/usr/bin/env python

import sys
import rospy
import cv2 
import numpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionFeedback

class assignment_map:
    
    green_min = numpy.array([50,150,50])
    green_max = numpy.array([100,255,255])
    blue_min = numpy.array([100,150,50])
    blue_max = numpy.array([150,255,255])
    red_min = numpy.array([0,200,100])
    red_max = numpy.array([5,255,255])
    yellow_min = numpy.array([30,200,100])
    yellow_max = numpy.array([50,255,195])
    
    wheel_radius = 0.076
    robot_radius = 0.43
    
    h = 0
    w = 0
    
    foundRed = False
    foundYellow = False
    foundBlue = False
    foundGreen = False
    
    laser_data = [0]
    laser_max = 10.0
    laser_min = 0.0
    
    hsv_img = []
    mask_img = []
    
    amcl = []
    feedback_data = []
    
    def __init__(self):
        cv2.namedWindow("Image window", 1)
        cv2.namedWindow("Thresh", 1)
        self.bridge = CvBridge()
        cv2.startWindowThread()
        self.motion_pub = rospy.Publisher("/turtlebot/cmd_vel", Twist, queue_size=10)
        self.move_base_pub = rospy.Publisher("turtlebot/move_base_simple/goal", PoseStamped, queue_size=10)
        self.move_base_feedback_pub = rospy.Subscriber("turtlebot/move_base/feedback", MoveBaseActionFeedback, self.update_feedback)        
        self.laser_sub = rospy.Subscriber("/turtlebot/scan/", LaserScan, self.update_laser_data)
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw", Image, self.image_raw)
        self.position_sub = rospy.Subscriber("/turtlebot/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose)        
        
    def update_laser_data(self, data):
        self.laser_data = data.ranges
        self.laser_max = data.range_max
        self.laser_min = data.range_min
    
    def update_feedback(self, data):
        self.feedback_data = data
        
    def image_raw(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # define range of green color in HSV
        self.h, self.w, c = self.hsv_img.shape
        
        # Threshold the HSV image to get only pole colors
        self.mask_img = self.threshold_image()
        
        #Remove top and bottom of the image
        #mask_img[0:self.h/4, 0:self.w] = 0
        #mask_img[self.h - (self.h/4):self.h, 0:self.w] = 0
        
        #self.naive_behaviour()        
        self.display(cv_image)
    
    def amcl_pose(self, data):
        self.amcl = data
        
    def threshold_image(self):
        mask = numpy.zeros((self.h, self.w, 1), dtype="uint8")
        
        if(not self.foundGreen):
            mask = cv2.inRange(self.hsv_img, self.green_min, self.green_max)
        
        if(not self.foundBlue):        
            mask = cv2.bitwise_or(cv2.inRange(self.hsv_img, self.blue_min, self.blue_max), mask)

        if(not self.foundRed):           
            mask = cv2.bitwise_or(cv2.inRange(self.hsv_img, self.red_min, self.red_max), mask)
            
        if(not self.foundYellow):   
            mask = cv2.bitwise_or(cv2.inRange(self.hsv_img, self.yellow_min, self.yellow_max), mask)
        
        return mask
        
    def display(self, img):
        cv2.imshow("Image window", img)
        drawn_img = self.mask_img
        cv2.putText(drawn_img,'Red:{} Yellow:{} Blue:{} Green:{}'.format(self.foundRed, self.foundYellow, 
                    self.foundBlue, self.foundGreen), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.85, 255)
        cv2.imshow("Thresh", cv2.bitwise_and(img, img, mask=drawn_img))
            
            
    def f_kinematics(self, w_l, w_r):
        c_l = self.wheel_radius * w_l
        c_r = self.wheel_radius * w_r
        v = (c_l + c_r) / 2
        a = (c_l - c_r) / self.robot_radius
        return (v, a)   
    
    def check_object(self, image):
        mid_x = int(round(self.w/2))
        mid_y = int(round(self.w/2))
        
        sub_mat = image[mid_y-1:mid_y+1, mid_x-100:mid_x+100]
        
        #sub_mat = image[int(round(self.w/2))-100:int(round(self.w/2))+100,int(round(self.h/2))-100:int(round(self.h/2))+100]
        
        min_size = 100 
        
        if(cv2.inRange(sub_mat, self.green_min, self.green_max).sum() > min_size):
            self.foundGreen = True
            return self.found_object()
        elif(cv2.inRange(sub_mat, self.blue_min, self.blue_max).sum() > min_size):
            self.foundBlue = True
            return self.found_object()
        elif(cv2.inRange(sub_mat, self.red_min, self.red_max).sum() > min_size):
            self.foundRed = True
            return self.found_object()
        elif(cv2.inRange(sub_mat, self.yellow_min, self.yellow_max).sum() > min_size):
            self.foundYellow = True
            return self.found_object()
            
        return False
    
    def move_to(self, x, y, orientation):
        ps = PoseStamped()
        ps.header.frame_id = "/map"
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0
        ps.pose.orientation = orientation
        self.move_base_pub.publish(ps)
        
    def found_object(self):
        m = Twist()
        m.linear.x = m.linear.y = m.angular.z = 0
        self.motion_pub.publish(m)
        print "Found object!"
        rospy.sleep(2)
        return True

    def naive_behaviour(self):
        v = 0 #Set initial velocity and angular
        a = 0
        
        v_speed = 1 #Set variaboles for v, a speeds
        a_speed = 0.8
        d_min = 1 #Set a variable for min distance to get to objects
        
        cmd_vel = Twist()
        cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0
        
        ranges = numpy.array(self.laser_data, dtype=numpy.float)
        ranges[numpy.isnan(ranges)] = self.laser_max
        
        midpoint = int(round(len(ranges) / 2))  
        lr_len = int(round(len(ranges) / 2)) #45* of data
        
        left_to_mid = ranges[midpoint-lr_len + 1:midpoint]
        mid_to_right = ranges[midpoint:midpoint+lr_len - 1]
        
        left_range = min(left_to_mid) if len(left_to_mid) > 0 else 0   
        center_range = ranges[midpoint] if ranges[midpoint] else 0
        right_range = min(mid_to_right) if len(mid_to_right) > 0 else 0
        
        left_average = numpy.mean(left_to_mid)
        right_average = numpy.mean(mid_to_right)
        
        print left_range, center_range, right_range
                    
        if(self.mask_img.sum() >= 1):
            M = cv2.moments(self.mask_img)

            if(center_range < 0.9):
                if(not self.check_object(self.hsv_img)):
                    a = a_speed
            elif M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                v = v_speed
                a = (-float(cx - self.w/2) / 100)
                
        elif left_range > d_min and right_range > d_min:
            direction_min = min([left_range, center_range, right_range])
            v = v_speed if direction_min > d_min else -v_speed
        elif left_range - right_range < -d_min:
            a = a_speed
        elif left_range - right_range > +d_min:
            a = -a_speed
        else:
            a = a_speed
        
        cmd_vel.linear.x = v if v <= v_speed else v_speed
        cmd_vel.angular.z = a if a <= v_speed else a_speed
        
        self.motion_pub.publish(cmd_vel)

def main():
    rospy.init_node('assignment_one')
    tb = assignment_map()
    pos = [0, 0]

    while(True):
        print tb.feedback_data
        rospy.sleep(1)
    
    rospy.spin()
        
if __name__ == '__main__':    
    main()
    cv2.destroyAllWindows()