#! /usr/bin/env python

import rospy
import math
import actionlib
import actionlib.msg
import assignment_2_2022.msg
import time
import sys
import select
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from assignment_2_2022.msg import Robot_pos_vel



def callback(msg):
	
    # Get the desired position
    des_pos_x = rospy.get_param("des_pos_x")
    des_pos_y = rospy.get_param("des_pos_y")
        
    # Get the actual position and speed
    v_x = msg.vel_x
    v_y = msg.vel_y
    x = msg.pos_x
    y = msg.pos_y
    
        
    # Compute the distance
    dist = ((x-des_pos_x)**2 + (y-des_pos_y)**2)**0.5
    
    # Compute the average speed
    avg_speed = (v_x**2 + v_y**2)**0.5
    
    # Get frequency parameter
    freq = rospy.get_param("/set_frequency")
     
    # Print  
    print("The distance from the goal is: " , dist)
    print("The average speed is: ", avg_speed)
    
    
    # Sleep time depend on frequency  
    time.sleep(1/freq)
    
    

def main():

    # Initializes a rospy node
    rospy.init_node('speed_distance', anonymous=True)
    
    # Subscriber to /pos_and_vel topic to get the position and velocity
    sub = rospy.Subscriber('/robot_pos_vel', Robot_pos_vel, callback)
	
    # Wait
    rospy.spin()
    
    
	
if __name__ == "__main__":
	main()	
