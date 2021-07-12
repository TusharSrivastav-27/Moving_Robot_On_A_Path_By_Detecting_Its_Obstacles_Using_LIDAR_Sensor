#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point
import math
from math import atan2
from math import sin

roll = pitch = yaw = 0.0
P = 1.4
x = 0.0
y = 0.0
count = 0.0
 
global goal
goal=Point()

def laser_callback(msg):
    global regions
    regions = {
        'bright':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'bleft':   min(min(msg.ranges[576:713]), 10),
    }


def Waypoints(n):
    global goal_x
    global goal_y
    goal_x=[0.65,1.23,1.78,2.46,3.14,3.78,5.00,5.62,6.00,12.50]
    goal_y=(2*(math.sin(goal_x[n])))*(math.sin((goal_x[n]/2)))
    goal.x = round(goal.x,2)
    goal.y = round(goal.y,2)

    

def odom_callback(data):
    global x
    global y
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    global roll, pitch, yaw
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

def obstacle():
    global count
    if(regions['front']<2):
        command.linear.x = 0.0
        pub.publish(command)
    
    while x<11.5:
        if regions['front'] < 1.5 and regions['fleft'] < 1.5 and regions['fright'] < 1.5:
            command.angular.z = 0.8
            command.linear.x = 0.0
            count = count + 1
            rospy.loginfo_once(regions)
            pub.publish(command)
        elif regions['front'] > 1.5 and regions['fleft'] > 2.0 and regions['fright'] < 1.5:
            command.linear.x = 0.4
            command.angular.z = 0.0
            rospy.loginfo_once(regions)
            pub.publish(command)
        elif regions['front'] > 1.5 and regions['fleft'] > 1.5 and regions['fright'] > 1.5:
            if(count==0):
                command.angular.z = 0.0
                command.linear.x = 0.3
                rospy.loginfo_once(regions)
                pub.publish(command)
            else:
                command.angular.z = -0.6
                command.linear.x = 0.2
                pub.publish(command)
        else:
            pass





rospy.init_node('ebot_controller')

sub = rospy.Subscriber ('/odom', Odometry, odom_callback)
rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

command= Twist()
rate = rospy.Rate(4)

def move(pts):
    Waypoints(pts)
    goal.x=goal_x[pts]
    goal.y = goal_y
    inc_x = goal.x - x
    inc_y = goal.y - y
    target_angle = atan2(inc_y,inc_x)
    command.angular.z = P  * (target_angle - yaw)
    command.linear.x = 0.2


def control_loop():
    
    while not rospy.is_shutdown(): 
       move(0)
       if x >= goal.x:
           move(1)
       if x >= goal.x:
           move(2)
       if x >= goal.x:
           move(3)
       if x >= goal.x:
           move(4)
       if x >= goal.x:
           move(5)
       if x >= goal.x:
           move(6)
       if x >= goal.x:
           move(7)
       if x >= goal.x:
           Waypoints(8)
           goal.x=goal_x[8]
           goal.y = goal_y
           inc_x = goal.x - x
           inc_y = goal.y - y
           target_angle = atan2(inc_y,inc_x)
           command.angular.z = 0.9  * (target_angle - yaw)
           command.linear.x = 0.2
       if x >= goal.x:
           Waypoints(9)
           goal.x=goal_x[9]
           goal.y = goal_y
           inc_x = goal.x - x
           inc_y = goal.y - y
           target_angle = atan2(inc_y,inc_x)
           command.angular.z = 0.9  * (target_angle - yaw)
           command.linear.x = 0.2
       if x >= 8.0: 
           command.linear.x=0.0
           obstacle() 
       if x >= 11.5:
           Waypoints(9)
           goal.x=goal_x[9]
           goal.y = goal_y
           inc_x = goal.x - x
           inc_y = goal.y - y
           target_angle = atan2(inc_y,inc_x)
           command.angular.z = 0.9  * (target_angle - yaw)
           command.linear.x = 0.2
           if x >= goal.x:
                command.linear.x = 0.0
                command.angular.z = 0.0 
       pub.publish(command)
       print("Controller message pushed at {}".format(rospy.get_time()))
       rate.sleep() 
                
  

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass

    






          
 

                            



                
       
  



