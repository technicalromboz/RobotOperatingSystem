#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time

def takeoff_and_move_in_square():
    # Initialize the ROS node
    rospy.init_node('drone_takeoff_and_square_motion_node', anonymous=True)
    
    # Create a publisher to send takeoff command
    takeoff_pub = rospy.Publisher('/takeoff', Empty, queue_size=1)
    
    # Wait for the publisher to connect
    rospy.sleep(1)
    
    # Send takeoff command
    takeoff_msg = Empty()
    takeoff_pub.publish(takeoff_msg)
    
    # Wait for the drone to stabilize
    rospy.sleep(5)  # Adjust the duration as needed
    
    rospy.loginfo("Drone has taken off and is hovering.")
    
    # Delay for 10 seconds before starting the square motion
    rospy.loginfo("Waiting for 10 seconds before starting square motion...")
    rospy.sleep(10)
    
    # Create a publisher to send velocity commands  
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # Create a Twist message to control the drone's motion
    cmd = Twist()
    
    # Define the side length of the square (in meters)
    side_length = 2.0
    
    # Set the linear velocity for forward motion
    cmd.linear.x = 0.2  # Adjust the speed as needed for smooth motion
    
    # Set the angular velocity for turning (to make a square)
    cmd.angular.z = 0.2  # Adjust the turning speed as needed for smooth motion
    
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
    
    # Add code here to land the drone if needed
    # Create a publisher to send land command
    land_pub = rospy.Publisher('/land', Empty, queue_size=1)
    
    # Wait for the publisher to connect
    rospy.sleep(1)
    
    # Send land command
    land_msg = Empty()
    land_pub.publish(land_msg)
    
    rospy.sleep(5)  # Wait for the drone to land
    
    rospy.loginfo("Drone is landing.")

if __name__ == '__main__':
    try:
        takeoff_and_move_in_square()
        rospy.spin()  # Keep the node alive until explicitly stopped
    except rospy.ROSInterruptException:
        pass
