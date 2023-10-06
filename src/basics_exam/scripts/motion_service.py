#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist

rospy.init_node('motion_service_node')

def handle_motion_service(req):
    rospy.loginfo("Starting the square motion...")

    # Create a publisher to send velocity commands
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    cmd = Twist()

    # Define the side length of the square (in meters)
    side_length = 2.0

    # Set the linear velocity for forward motion
    cmd.linear.x = 0.5

    # Set the angular velocity for turning (to make a square)
    cmd.angular.z = 0.5

    # Calculate the time needed to travel one side of the square
    travel_time = side_length / cmd.linear.x

    # Loop to move the drone in a square
    for _ in range(4):
        cmd_vel_pub.publish(cmd)  # Move forward
        rospy.sleep(travel_time)  # Wait for the drone to reach the next corner

        # Stop the drone
        cmd_vel_pub.publish(Twist())  # Empty Twist message to stop the drone
        rospy.sleep(1)  # Pause at each corner

    rospy.loginfo("Drone has completed the square motion.")

    # Return an EmptyResponse to indicate success
    return EmptyResponse()

rospy.Service('/motion_service', Empty, handle_motion_service)
