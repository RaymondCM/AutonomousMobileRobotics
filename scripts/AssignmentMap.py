#!/usr/bin/env python

import rospy
import cv2 
import numpy
import time
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID

class assignment_map:
    colour_thresholds = {'Red': {
                        'min': numpy.array([0,200,100]),
                        'max': numpy.array([5,255,255]),
                        'found': False
                        },
                        'Green':{
                        'min': numpy.array([50,150,50]),
                        'max': numpy.array([100,255,255]),
                        'found': False
                        },
                        'Yellow':{
                        'min': numpy.array([30,200,100]),
                        'max': numpy.array([50,255,195]),
                        'found': False
                        },
                        'Blue':{
                        'min': numpy.array([100,150,50]),
                        'max': numpy.array([150,255,255]),
                        'found': False
                        }}
                             
    h = 0
    w = 0
    
    laser_data = [0]
    laser_max = 10.0
    laser_min = 0.0
    
    hsv_img = []
    mask_img = []
    
    amcl = []
    
    points = [[-2, -4, 0.004], [-0.9, 3.15, 0.007], [1.15, -0.5, -0.001], [-4, 1.53, 0.004]]
    current_point = 0
    searching = False
    has_scanned = False
    scan_time = time.time()
    found_an_object = False
    
    def __init__(self):
        cv2.namedWindow("Image window", 1)
        cv2.namedWindow("Thresh", 1)
        self.bridge = CvBridge()
        cv2.startWindowThread()
        self.motion_pub = rospy.Publisher("/turtlebot/cmd_vel", Twist, queue_size=10)
        self.move_base_pub = rospy.Publisher("turtlebot/move_base_simple/goal", PoseStamped, queue_size=10)
        self.cancel_move_base_pub = rospy.Publisher("/turtlebot/move_base/cancel", GoalID, queue_size=10)        
        
        self.laser_sub = rospy.Subscriber("/turtlebot/scan/", LaserScan, self.update_laser_data)
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw", Image, self.update_image_data)
        self.position_sub = rospy.Subscriber("/turtlebot/amcl_pose", PoseWithCovarianceStamped, self.update_amcl_data)        
        rospy.sleep(2)
        
    def update_laser_data(self, data):
        self.laser_data = data.ranges
        self.laser_max = data.range_max
        self.laser_min = data.range_min
    
    def update_amcl_data(self, data):
        self.amcl = data
   
    def update_image_data(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        self.h, self.w, c = self.hsv_img.shape
        self.mask_img = self.threshold_image()
        
        if self.searching:
            self.explore()
            
        self.display(cv_image)
    
    def threshold_image(self):
        mask = numpy.zeros((self.h, self.w, 1), dtype="uint8")
        
        for c in self.colour_thresholds:
            c_obj = self.colour_thresholds[c]
            if not c_obj['found']:
               mask = cv2.bitwise_or(cv2.inRange(self.hsv_img, c_obj['min'], c_obj['max']), mask)

        return mask
        
    def display(self, img):
        drawn_img = self.mask_img
        
        cv2.putText(drawn_img,'Red:{} Yellow:{} Blue:{} Green:{}'.format(
            self.colour_thresholds['Red']['found'], 
            self.colour_thresholds['Yellow']['found'],  
            self.colour_thresholds['Blue']['found'],  
            self.colour_thresholds['Green']['found']),
            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.85, 255)
        
        cv2.imshow("Image window", img)
        cv2.imshow("Thresh", cv2.bitwise_and(img, img, mask=drawn_img))
    
    def found_all(self):
        for c in self.colour_thresholds:
            if not self.colour_thresholds[c]['found']:
                return False
        
        return True
        
    def check_object(self, image):
        mid_x = int(round(self.w/2))
        mid_y = int(round(self.w/2))
        
        sub_mat = image[mid_y-1:mid_y+1, mid_x-100:mid_x+100]
        min_size = 100
        
        for c in self.colour_thresholds:
            c_obj = self.colour_thresholds[c]
            if(cv2.inRange(sub_mat, c_obj['min'], c_obj['max']).sum() > min_size):
                self.colour_thresholds[c]['found'] = True
                return self.found_object()
                        
        return False
    
    def move_to(self, x, y, z):
        ps = PoseStamped()
        ps.header.frame_id = "/map"
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0
        ps.pose.orientation = self.amcl.pose.pose.orientation
        ps.pose.orientation.z = 0
        print self.amcl.pose.pose.orientation
        self.move_base_pub.publish(ps)
    
    def get_goal_pos(self):
        i = self.current_point
        return self.points[i][0], self.points[i][1], self.points[i][2]
    
    def is_close_to_goal(self, min_distance):
        g_x, g_y, g_z = self.get_goal_pos()
        x_diff = abs(self.amcl.pose.pose.position.x - g_x)
        y_diff = abs(self.amcl.pose.pose.position.y - g_y)
        z_diff = abs(self.amcl.pose.pose.position.z - g_z)
        return x_diff <= min_distance and y_diff <= min_distance and z_diff <= min_distance
            
    def move_next(self):
        self.found_an_object = False
        if self.current_point == len(self.points):
            self.current_point = 0

        self.move_to(*self.get_goal_pos())
        
    def is_at_position(self):
        try:
            at_pos = self.is_close_to_goal(0.3)
            
            if(at_pos):
                print "I'm at the positon"
                self.current_point += 1
            
            return at_pos
        except Exception:
            return False
    
    def cancel_move_base(self):
        a = GoalID()
        self.cancel_move_base_pub.publish(a)
    
    def object_search(self): 
        self.has_scanned = False
        self.scan_time = time.time();
        self.searching = True
        return True
        
    def go_to_next(self):
        if(self.found_an_object):
            return True
        else:
            self.found_an_object = False
            return False
            
    def cancel_motion(self):
        m = Twist()
        m.linear.x = m.linear.y = m.angular.z = 0
        self.motion_pub.publish(m)
        
    def found_object(self):
        self.cancel_motion()
        print "Found object!"
        self.found_an_object = True
        self.searching = False
        rospy.sleep(2)
        return True

    def explore(self):
        print "Behaviour on"
        self.has_scanned = abs(self.scan_time - time.time()) >= 8 #Change to a period of time rather than count
        self.searching = abs(self.scan_time - time.time()) <= 20
        
        v = 0 #Set initial velocity and angular
        a = 0
        
        v_speed = 0.3 #Set variaboles for v, a speeds
        a_speed = 1
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
                    
        if(self.mask_img.sum() >= 1):
            M = cv2.moments(self.mask_img)

            if(center_range < 0.9):
                if(not self.check_object(self.hsv_img)):
                    a = a_speed
            elif M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                v = v_speed
                a = (-float(cx - self.w/2) / 100)
                
        elif not self.has_scanned:
            a = a_speed
        elif left_range > d_min and right_range > d_min:
            direction_min = min([left_range, center_range, right_range])
            v = v_speed if direction_min > d_min else -v_speed
        elif left_range - right_range < -d_min:
            a = a_speed
        elif left_range - right_range > +d_min:
            a = -a_speed
        else:
            a = -a_speed
        
        cmd_vel.linear.x = v if v <= v_speed else v_speed
        cmd_vel.angular.z = a if a <= v_speed else a_speed
        
        self.motion_pub.publish(cmd_vel)

def main():
    rospy.init_node('assignment_one')
    tb = assignment_map()
    
    sentGoal = False    
    while(not tb.found_all()):
        
        if(sentGoal == False):
            print "Sending goal"
            tb.move_next()
            sentGoal = True
            
        if(tb.is_at_position()):
            print "At position"
            tb.cancel_move_base()
            rospy.sleep(1)
            tb.object_search()
        else:
            print "Searching:", tb.go_to_next()
            sentGoal = not tb.go_to_next()
        
        rospy.sleep(1)
    
    rospy.spin()
        
if __name__ == '__main__':    
    main()
    cv2.destroyAllWindows()
    
#        def object_search(self):
#        self.initial_spin()
#        return True
#    
#    def turn_until(self, i):
#        m = Twist()
#        m.linear.x = 0
#        m.angular.z = 0.5
#        
#        if i >= 10:
#            return True
#            
#        if(self.mask_img.sum >= 100):
#            m.angular.z = 0
#            self.motion_pub.publish(m)
#            return True
#            
#        rospy.sleep(2)
#        self.turn_until(i)
#        
#
#    def initial_spin(self):
#        self.searching = turn_until()
#        return False