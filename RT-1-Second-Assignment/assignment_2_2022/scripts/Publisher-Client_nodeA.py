#! /usr/bin/env python

import math
import actionlib
import actionlib.msg
import assignment_2_2022.msg
import rospy
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

    global pub
 
    # Get the position 
    pos = msg.pose.pose.position
    
    # Get the linear velocity
    linear_velocity = msg.twist.twist.linear
    
    # Create custom message
    robot_pos_vel = Robot_pos_vel()
    robot_pos_vel.pos_x = pos.x
    robot_pos_vel.pos_y = pos.y
    robot_pos_vel.vel_x = linear_velocity.x
    robot_pos_vel.vel_y = linear_velocity.y
    
    # Publish the custom message
    pub.publish(robot_pos_vel)
        
        
        
def Client():
    
    # Creates the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction)

    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Goal stat is true if the robot is reaching the position otherwise is false
    goal_stat = False
	
    while not rospy.is_shutdown():
        
        # Get the keyboard inputs
        print("Please insert a new position or type c to cancel it ")
        x = input("x: or c: ")
        y = input("y: or c: ")
        
 	# If user entered 'c' and the robot is reaching the goal position, cancel the goal
        if x == "c" and goal_stat == True:
            
            # Cancel the goal
            client.cancel_goal()
            goal_stat = False

        else:
            # Convert numbers from string to float
            x = float(x)
            y = float(y)
            
            # Create the goal to send to the server
            goal = assignment_2_2022.msg.PlanningGoal()

            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
					
            # Send the goal to the action server
            client.send_goal(goal)
            
            goal_stat = True


       
def main():

    global pub
    
    try:
        # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS.
        rospy.init_node('action_client_py')
        
        # Publisher to /pos_and_vel topic the position and velocity
        pub = rospy.Publisher("/robot_pos_vel", Robot_pos_vel, queue_size=10)
        
        # Subscriber to /odom topic to get position and velocity
        sub = rospy.Subscriber('/odom', Odometry, callback)
        
        # Start client
        Client()
        
               
    except rospy.ROSInterruptException:
        print("Error has occurred")
        
        
   
if __name__ == '__main__':
    main()
