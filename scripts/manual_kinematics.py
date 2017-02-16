import math
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

wheel_radius = 0.076
robot_radius = 0.43

# computing the forward kinematics for a differential drive
def forward_kinematics(w_l, w_r):
    c_l = wheel_radius * w_l
    c_r = wheel_radius * w_r
    v = (c_l + c_r) / 2
    a = (c_l - c_r) / robot_radius
    return (v, a)


# computing the inverse kinematics for a differential drive
def inverse_kinematics(v, a):
    c_l = v + (robot_radius * a) / 2
    c_r = v - (robot_radius * a) / 2
    w_l = c_l / wheel_radius
    w_r = c_r / wheel_radius
    return (w_l, w_r)


# inverse kinematics from a Twist message (This is what a ROS robot has to do)
def inverse_kinematics_from_twist(t):
    return inverse_kinematics(t.linear.x, t.angular.z)

class manual_kinematics:
    
    def __init__(self):
        
        self.wheel_vel_sub = rospy.Subscriber("wheel_vel_left", Float32, self.wheel_vel_callback)
        self.cmd_vel = rospy.Publisher("/turtlebot_1/cmd_vel", Twist)
        
    def wheel_vel_callback(self, data):
        t = Twist()
        (t.linear.x, t.angular.z) = forward_kinematics(float(data.data), 0.0)
        
        print("X: ", t.linear.x, " Z: ", t.angular.z)

#Start rosnode        
rospy.init_node('manual_kinematics')
mk = manual_kinematics()

#Keep process from closing until node stops
rospy.spin()