#!/usr/bin/env python

#-------------------------------------------------------#
#Raymond Kirk KIR14474219 14474219@students.lincoln.ac.uk
#-------------------------------------------------------#

#Import all dependancies
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
    #Create object that defines minimum and maximum ranges to only threshold
    # each pole and to store if it has already been found.
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
    
    #Create class variables to store default values                         
    h = 0
    w = 0
    
    laser_data = [0]
    laser_max = 10.0
    laser_min = 0.0
    
    hsv_img = []
    mask_img = []
    
    amcl = []
    
    #Set max and minimum values
    v_speed = 0.3
    a_speed = 1
    d_min = 1
    
    #Define points to go to on the map and search
    points = [[-2, -4, 0], [-0.9, 3.15, 0], [1.15, -0.5, 0], [-4, 1.53, 0]]
    current_point = 0
    
    #Boolean flags to determine the behaviour of the robot
    ## searching determines if the robot is allowed to explore
    ## has_scanned determines if the robot has taken in it's surroundings
    ## scan_time helps track what the robot has been doing (simple time-point)
    ## found_an_object determines when to stop asyncronous behaviour
    searching = False
    has_scanned = False
    scan_time = time.time()
    found_an_object = False
    
    def __init__(self):
        #Create windows for imshow to render pixel data within
        ## Define bridge between opencv and imgmsg to process frames
        ## start window on seperate thread to not lock up main process
        cv2.namedWindow("Image window", 10)
        cv2.namedWindow("Thresh", 1)
        self.bridge = CvBridge()
        cv2.startWindowThread()
        
        #Define publishers
        ## Motion_Pub for manually publishing linear and angular data
        ## move_base_pub publishes points to the navigation stack and proceeds to that point
        ## cancel_move_base cancels any current path plan
        self.motion_pub = rospy.Publisher("/turtlebot/cmd_vel", Twist, queue_size=10)
        self.move_base_pub = rospy.Publisher("turtlebot/move_base_simple/goal", PoseStamped, queue_size=1)
        self.cancel_move_base_pub = rospy.Publisher("/turtlebot/move_base/cancel", GoalID, queue_size=1)        
        
        #Define subscribers
        ## laser_sub provides information on obstacles < min and > max distance
        ## image_sub updates the current frame information and calls thresholding functions
        ## position sub keeps track of position on the map, useful for determining
        ##  euclidean distance/threshold to position
        self.laser_sub = rospy.Subscriber("/turtlebot/scan/", LaserScan, self.update_laser_data)
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw", Image, self.update_image_data)
        self.position_sub = rospy.Subscriber("/turtlebot/amcl_pose", PoseWithCovarianceStamped, self.update_amcl_data)        
        
        #Sleep for two seconds, allowing pub/subs to fully initialise        
        rospy.sleep(2)
    
    #Update current laser data to the latest available
    def update_laser_data(self, data):
        self.laser_data = data.ranges
        self.laser_max = data.range_max
        self.laser_min = data.range_min
    
    #Update current position data to the latest available
    def update_amcl_data(self, data):
        self.amcl = data
   
   #Update current image data to the latest available and ask if it can
   ## behave/explore the enviroment. Display results.
    def update_image_data(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        self.h, self.w, c = self.hsv_img.shape
        self.mask_img = self.threshold_image()
        
        self.ask_to_explore()
        
        self.display(cv_image)
    
    #Explores if the robot is in searching mode
    def ask_to_explore(self):
        if self.searching:
            self.explore()
    
    #Loop over thresholding information and produce a mask image
    def threshold_image(self):
        mask = numpy.zeros((self.h, self.w, 1), dtype="uint8")
        
        for c in self.colour_thresholds:
            c_obj = self.colour_thresholds[c]
            if not c_obj['found']:
               mask = cv2.bitwise_or(cv2.inRange(self.hsv_img, c_obj['min'], c_obj['max']), mask)

        return mask
    
    #Display current information and overlay on the output image
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
    
    #Function to determine if all objects have been found, stops executuion of syncronous loop
    def found_all(self):
        for c in self.colour_thresholds:
            if not self.colour_thresholds[c]['found']:
                return False
        
        return True
        
    #Thresholds the hsv_image and checks if an object matches one of the unfound
    ## objects, then confirms the size is of a certain size before saying it's been found
    ## is only called if the center range to an object is less than a meter
    def check_object(self, image):
        mid_x = int(round(self.w/2))
        mid_y = int(round(self.w/2))
        
        #Get a scan line from the center of the image
        sub_mat = image[mid_y-1:mid_y+1, mid_x-100:mid_x+100]
        min_size = 100
        
        #Check and classify objects then call found_object if found.
        for c in self.colour_thresholds:
            c_obj = self.colour_thresholds[c]
            if(cv2.inRange(sub_mat, c_obj['min'], c_obj['max']).sum() > min_size):
                self.colour_thresholds[c]['found'] = True
                return self.found_object(c)
                        
        return False
                
    #sets seaching to false, stopping map exploration
    def found_object(self, colour):
        #Cancel all motion (Twist())
        self.cancel_motion()
        self.found_an_object = True
        self.searching = False
        
        #Rest for two seconds and alert the user of which object was found        
        print "Found", colour, "object!"
        rospy.sleep(2)
        return True
    
    #Publishes 0 linear and angular motion to overide current motion
    def cancel_motion(self):
        m = Twist()
        m.linear.x = m.linear.y = m.angular.z = 0
        self.motion_pub.publish(m)
        
    #Called when the robot is allowed to explore, contains logic that can safely traverse
    ## the enviroment for 20 seconds, including a scan phase to check surroundings
    ## uses laser data to avoid objects and odometry errors, attempts to move into open space
    ## and or rotate into available gaps. Alters angular speed to center detected objects.
    def explore(self):

        self.has_scanned = abs(self.scan_time - time.time()) >= 8 #Change to a period of time rather than count
        self.searching = abs(self.scan_time - time.time()) <= 20
        
        #Set initial values for angular and velocity speeds
        v = 0 
        a = 0
        
        #Initialise twist message with 0 values
        cmd_vel = Twist()
        cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0
        
        #Calculate laser_data and split into segments of left, right and center
        ## Replace all nan values with laser_max to stop incorrect collision/ max range data
        ## Aditional check if information exists in laser_data
        ranges = numpy.array(self.laser_data, dtype=numpy.float)
        ranges[numpy.isnan(ranges)] = self.laser_max
        
        midpoint = int(round(len(ranges) / 2))  
        lr_len = int(round(len(ranges) / 2)) #45* of data
        
        left_to_mid = ranges[midpoint-lr_len + 1:midpoint]
        mid_to_right = ranges[midpoint:midpoint+lr_len - 1]
        
        left_range = min(left_to_mid) if len(left_to_mid) > 0 else 0   
        center_range = ranges[midpoint] if ranges[midpoint] else 0
        right_range = min(mid_to_right) if len(mid_to_right) > 0 else 0

        #If the thresholded image is displaying an object calculate the moments
        ## and center of mass, correct current angular.z to align with the object
        ## if the object is less than a meter range then check if it is a valid object
        if(self.mask_img.sum() >= 1):
            M = cv2.moments(self.mask_img)
            
            if(center_range < 0.9):
                #Check if the object is valid if not reverse (never will be true in sim)
                if(not self.check_object(self.hsv_img)):
                    a = -self.a_speed
            elif M['m00'] > 0:
                #Calculate centroid of mass and correct to align with it
                cx = int(M['m10']/M['m00'])
                v = self.v_speed
                a = (-float(cx - self.w/2) / 100)
        #Else if in scanning phase continue to explore 360 around the robot
        elif not self.has_scanned:
            a = self.a_speed
        #If there is a space forward move into it
        elif left_range > self.d_min and right_range > self.d_min:
            direction_min = min([left_range, center_range, right_range])
            v = self.v_speed if direction_min > self.d_min else -self.v_speed
        #If there is space on the right but not on the left turn right
        elif left_range - right_range < -self.d_min:
            a = self.a_speed
        #If there is space on the left but not on the right turn left
        elif left_range - right_range > +self.d_min:
            a = -self.a_speed
        #If all else fails turn on the spot until some is seen
        else:
            a = -self.a_speed
        
        #Limit speeds to global maximums and minimums
        cmd_vel.linear.x = v if v <= self.v_speed else self.v_speed
        cmd_vel.angular.z = a if a <= self.v_speed else self.a_speed
        
        self.motion_pub.publish(cmd_vel)
    
    #Create PoseStamped message and publish to topic to plan route to the point
    def move_to(self, x, y, z):
        ps = PoseStamped()
        ps.header.frame_id = "/map"
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0
        #Use orientation that the robot initially had
        ps.pose.orientation = self.amcl.pose.pose.orientation
        ps.pose.orientation.z = 0
        
        self.move_base_pub.publish(ps)
    
    #get positions of current goal point in x, y, z
    def get_goal_pos(self):
        i = self.current_point
        return self.points[i][0], self.points[i][1], self.points[i][2]
    
    #Return a boolean value that describes if all the coordiantes are within a certain threshold
    def is_close_to_goal(self, min_distance):
        g_x, g_y, g_z = self.get_goal_pos()
        x_diff = abs(self.amcl.pose.pose.position.x - g_x)
        y_diff = abs(self.amcl.pose.pose.position.y - g_y)
        z_diff = abs(self.amcl.pose.pose.position.z - g_z)
        return x_diff <= min_distance and y_diff <= min_distance and z_diff <= min_distance
            
    #Move to next point in the array or loop back over the points
    def move_next(self):
        self.found_an_object = False
        if self.current_point == len(self.points):
            self.current_point = 0

        #Squash return into parameters
        self.move_to(*self.get_goal_pos())
    
    #Returns a boolean that describes if the robot is at the current position.
    ## if it is it increments that current_point by one, use 0.3 as closeness
    def is_at_position(self):
        try:
            at_pos = self.is_close_to_goal(0.3)
            
            if(at_pos):
                print "I'm at the positon"
                self.current_point += 1
            
            return at_pos
        except Exception:
            return False
    
    #Cancel current goal plan.
    def cancel_move_base(self):
        a = GoalID()
        self.cancel_move_base_pub.publish(a)
    
    #Configure enviroment to allow the robot to explore
    def object_search(self): 
        self.has_scanned = False
        self.scan_time = time.time();
        self.searching = True
        return True
    
    #Return a boolean describing if an object has been found.
    ## if it has then change found an object to false and reset.
    def go_to_next(self):
        if(self.found_an_object):
            return True
        else:
            self.found_an_object = False
            return False

#Syncronous behaviour of the robot
def main():
    ##Intialise the node and the class
    rospy.init_node('assignment_one')
    tb = assignment_map()
    
    #Create variable that tracks if the object is in move_base goal state
    moving_to_goal = False    
    while(not tb.found_all()):
        
        #If the object isn't moving to a goal, set it a new one
        if(moving_to_goal == False):
            print "Sending goal"            
            moving_to_goal = True
            #Call function to move to the next point
            tb.move_next()
        
        #If the robot reaches the position then log to console, cancel any remaining
        ## distance due to threshold and start search of an object allowing
        ## the robot to behave asyncronously
        if(tb.is_at_position()):
            print "At position"
            tb.cancel_move_base()
            ##Sleep for one second so transition between states is visible
            rospy.sleep(1)
            tb.object_search()
        else:
            ##Else set moving to goal to weather an object has been found so that
            ##The next point can be set
            moving_to_goal = not tb.go_to_next()
        
        ##Sleep to one second so not too many checks are made
        rospy.sleep(1)
    
    #Keeps rospy alive until program termination
    rospy.spin()
        
if __name__ == '__main__':    
    main()
    ##After main is about to terminate close all windows.
    cv2.destroyAllWindows()
    