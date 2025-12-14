#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np
import sys
import os
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped



ODOM_TOPIC = "/stickbug/drivebase/zed/zed_node/odom"



class StateEstimatorPublisher():
    def __init__(self):

        rospy.init_node("state_estimation_node",anonymous=True)
        rospy.loginfo("State Estimator  node is running")
        rospy.Subscriber(ODOM_TOPIC, Odometry, self.odometry_subscriber_callback)
        self.listener = tf.TransformListener()
        rospy.sleep(2.5)
        #t = self.listener.getLatestCommonTime("map", "odom")
        #print(t)

        self.odometry_publisher = rospy.Publisher("/state_estimation", Odometry, queue_size = 1)
        self.new_odom = Odometry()
        self.r = rospy.Rate(1) #control loop rate
        rospy.on_shutdown(self.shutdown)
        while not rospy.is_shutdown():
            self.r.sleep()


    def odometry_subscriber_callback(self,data):
        self.new_odom = data
        pose = PoseStamped()
        #pose.header.stamp = rospy.get_rostime()
        #pose.header.frame_id = "odom"
        pose.header = self.new_odom.header
        pose.pose = self.new_odom.pose.pose
        #t = self.listener.getLatestCommonTime("map", "odom")
        #if self.listener.frameExists("/odom") and self.listener.frameExists("/map"):
        t = self.listener.getLatestCommonTime("/odom", "/map")
        new_pose = self.listener.transformPose("map",pose)
        print(new_pose)

        self.new_odom.header = new_pose.header
        self.new_odom.pose.pose = new_pose.pose
        print(self.new_odom)

        self.odometry_publisher.publish(self.new_odom)
            

    def shutdown(self):
        rospy.loginfo("State Estimation Node is Shutdown")
        rospy.sleep(1)

def main():
    try:
        state_estimator = StateEstimatorPublisher()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()