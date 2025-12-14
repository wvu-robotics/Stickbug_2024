#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def cmd_vel_publisher():
    # Initialize the ROS node
    rospy.init_node('cmd_vel_publisher', anonymous=True)

    # Create a publisher for the /cmd_vel topic with Twist messages
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Create a Twist message to hold the velocity commands
    cmd_vel_msg = Twist()

    # Set linear velocity components (m/s)
    cmd_vel_msg.linear.x = 0.0  # Linear velocity in the x-direction
    cmd_vel_msg.linear.y = 0.0 # Linear velocity in the y-direction
    cmd_vel_msg.linear.z = 0.0# Linear velocity in the z-direction

    # Set angular velocity components (rad/s)
    cmd_vel_msg.angular.x = 5.0  # Angular velocity in the x-direction
    cmd_vel_msg.angular.y = 0.0 # Angular velocity in the y-direction
    cmd_vel_msg.angular.z = 0.0  # Angular velocity in the z-direction

    # Publish the Twist message at a specified rate (e.g., 10 Hz)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Publish the Twist message
        cmd_vel_pub.publish(cmd_vel_msg)

        # Sleep to maintain the specified rate
        rate.sleep()

if __name__ == '__main__':
    try:
        cmd_vel_publisher()
    except rospy.ROSInterruptException:
        pass