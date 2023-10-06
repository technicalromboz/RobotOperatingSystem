#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty

def takeoff():
    rospy.init_node('takeoff_node', anonymous=True)
    takeoff_pub = rospy.Publisher('/takeoff', Empty, queue_size=10)
    rospy.sleep(1)  # Wait for the publisher to initialize

    takeoff_msg = Empty()
    takeoff_pub.publish(takeoff_msg)

if __name__ == '__main__':
    try:
        takeoff()
    except rospy.ROSInterruptException:
        pass
