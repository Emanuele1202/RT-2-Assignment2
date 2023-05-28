#! /usr/bin/env python

import rospy
import math
import actionlib
import actionlib.msg
import assignment_2_2022.msg
import time
import sys
import select
from tf import transformations
from std_srvs.srv import *
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from assignment_2_2022.msg import Robot_pos_vel
from assignment_2_2022.srv import RC_Goal_num, RC_Goal_numResponse



# Variables initialization
canceled_goal_n = 0;
reached_goal_n = 0;



def callback(msg):

    global canceled_goal_n, reached_goal_n 

    # Get the status 
    status = msg.status.status

    # If status is 2 the goal is canceled
    if status == 2:
        canceled_goal_n = canceled_goal_n + 1

    # If status is 3 the goal is reached
    elif status == 3:
        reached_goal_n  = reached_goal_n  + 1
		


def track_goal_n(req):
    
    return  RC_Goal_numResponse(reached_goal_n , canceled_goal_n)



def main():
	
    # Initialize the node
    rospy.init_node('n_goal_rc_server')
	
    # Subscriber to /reaching_goal/result topic to get status
    sub = rospy.Subscriber('/reaching_goal/result', assignment_2_2022.msg.PlanningActionResult, callback)
    
    # Provide the service /n_goal
    s = rospy.Service('/goals_n', RC_Goal_num, track_goal_n)
    
    # Wait
    rospy.spin()
    
    

if __name__ == "__main__":
    main()
