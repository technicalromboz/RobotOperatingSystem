#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.laser_callback)

    def laser_callback(self, data):
        # Extract laser readings
        front_distance = data.ranges[len(data.ranges) // 2]
        right_distance = min(data.ranges[:len(data.ranges) // 4])
        left_distance = min(data.ranges[-len(data.ranges) // 4:])

        twist_cmd = Twist()
        
        if front_distance > 1.0:
            # No obstacle in front, move forward
            twist_cmd.linear.x = 0.2
            twist_cmd.angular.z = 0.0
        else:
            if left_distance < 1.0:
                # Obstacle at left, turn right
                twist_cmd.linear.x = 0.0
                twist_cmd.angular.z = -0.2
            elif right_distance < 1.0:
                # Obstacle at right, turn left
                twist_cmd.linear.x = 0.0
                twist_cmd.angular.z = 0.2
            else:
                # Obstacle in front, but not on sides, turn right
                twist_cmd.linear.x = 0.0
                twist_cmd.angular.z = -0.2
        
        self.cmd_vel_pub.publish(twist_cmd)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass