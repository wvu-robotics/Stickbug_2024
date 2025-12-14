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


ODOM_TOPIC = "/lio_sam/mapping/odometry"
#ODOM_TOPIC = "/stickbug/drivebase/zed/zed_node/odom"
CMD_VEL_TOPIC = "/stickbug/drivebase/autonomous_driving/cmd_vel"
WAYPOINT_TOPIC = "/way_point"
PATH_TOPIC = "/path"
WAYPOINT_DIST_THRESHOLD = 0.35
X_VEL = 0.65 # 0.7
Y_VEL = 0.3
Z_ANG_VEL = 0.2
PROPORTIONAL_GAIN = 1.0#0.7
INTEGRAL_GAIN = 0.1
LOOKAHEAD_DIS = 0.6
MAX_VEL = 1.0 #0.7
MAX_ANG_VEL = 0.12#0.25

class PathFollower():
    def __init__(self):

        rospy.init_node("path_follower_node",anonymous=True)
        rospy.loginfo("Path Follower node is running")
        rospy.Subscriber(PATH_TOPIC, Path, self.path_subscriber_callback)
        rospy.Subscriber(WAYPOINT_TOPIC, PointStamped, self.waypoint_subscriber_callback)
        rospy.Subscriber(ODOM_TOPIC, Odometry, self.odometry_subscriber_callback)
        self.cmd_vel_publisher = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size = 1)

        self.r = rospy.Rate(5) #control loop rate
        self.cmd_vel = Twist()
        rospy.on_shutdown(self.shutdown)
        self.move()

    def waypoint_subscriber_callback(self,data):
        self.waypoint_x = data.point.x
        self.waypoint_y = data.point.y
        self.waypoint_z = data.point.z
        print("New waypoint received")

        orientation_list = [self.odometry.pose.pose.orientation.x, self.odometry.pose.pose.orientation.y, self.odometry.pose.pose.orientation.z, self.odometry.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        print("Yaw: {}".format(np.rad2deg(yaw)))
        dist = np.linalg.norm(self.current_pos-np.array([self.waypoint_x,self.waypoint_y]))
        angle = np.arctan2(self.waypoint_y-self.current_pos[1],self.waypoint_x-self.current_pos[0])-yaw
        angle = np.arctan2(np.sin(angle),np.cos(angle))
        angle = np.rad2deg(angle)

        if np.abs(angle) > 45.0 and dist >1.0   :
            while np.abs(angle) > 25.0:
            # Turn in place
                print("Hi")
                self.cmd_vel = Twist()
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.linear.y = 0.0
                self.cmd_vel.angular.z = np.sign(angle)*MAX_ANG_VEL
                self.cmd_vel_publisher.publish(self.cmd_vel)
                orientation_list = [self.odometry.pose.pose.orientation.x, self.odometry.pose.pose.orientation.y, self.odometry.pose.pose.orientation.z, self.odometry.pose.pose.orientation.w]
                (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
                print("Yaw: {}".format(np.rad2deg(yaw)))
                angle = np.arctan2(self.waypoint_y-self.current_pos[1],self.waypoint_x-self.current_pos[0])-yaw
                angle = np.arctan2(np.sin(angle),np.cos(angle))
                angle = np.rad2deg(angle)


    def path_subscriber_callback(self,data):
        self.path = []
        self.path = data

    def odometry_subscriber_callback(self,data):
        self.odometry = data

    def move(self):
        self.odometry = rospy.wait_for_message(ODOM_TOPIC,Odometry)
        rospy.wait_for_message(PATH_TOPIC,Path)
        rospy.wait_for_message(WAYPOINT_TOPIC,PointStamped)

        old_vel_x = 0.0
        old_vel_y = 0.0
        old_ang_z = 0.0

        while not rospy.is_shutdown():
            
            self.current_pos = np.array([self.odometry.pose.pose.position.x,self.odometry.pose.pose.position.y])
            dist_x = 0
            dist_y = 0

            print(len(self.path.poses))
            for pose in self.path.poses:
                pose_ = np.array([pose.pose.position.x,pose.pose.position.y])
#                dist = np.linalg.norm(pose_-self.current_pos)
                dist = np.linalg.norm(pose_)
                dist_x = pose_[0] 
                dist_y = pose_[1] #+ VEHICLE_Y_REL#- self.current_pos[1]
                ang_z = np.arctan2(dist_y,dist_x)
                if dist > LOOKAHEAD_DIS:
                    break
                
            print("Dist X: {}, Dist Y: {}".format(dist_x,dist_y))
            vel_x = old_vel_x*INTEGRAL_GAIN + dist_x*PROPORTIONAL_GAIN
            vel_y = 0# old_vel_y*INTEGRAL_GAIN + dist_y*PROPORTIONAL_GAIN
            vel_ang_z = ang_z*PROPORTIONAL_GAIN # old_ang_z*INTEGRAL_GAIN +

            if vel_x > MAX_VEL:
                vel_x = MAX_VEL
            if vel_y > Y_VEL:
                vel_y = Y_VEL
            if vel_ang_z > MAX_ANG_VEL:
                vel_ang_z = MAX_ANG_VEL

            if vel_x < -MAX_VEL:
                vel_x = -MAX_VEL
            if vel_y < -Y_VEL:
                vel_y = -Y_VEL
            if vel_ang_z < -MAX_ANG_VEL:
                vel_ang_z = -MAX_ANG_VEL                

            old_vel_x = vel_x
            old_vel_y = vel_y
            old_ang_z = vel_ang_z

            # Calculate angle to waypoint.
            orientation_list = [self.odometry.pose.pose.orientation.x, self.odometry.pose.pose.orientation.y, self.odometry.pose.pose.orientation.z, self.odometry.pose.pose.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
            print("Yaw: {}".format(np.rad2deg(yaw)))
            angle = np.arctan2(self.waypoint_y-self.current_pos[1],self.waypoint_x-self.current_pos[0])-yaw
            angle = np.arctan2(np.sin(angle),np.cos(angle))
            angle = np.rad2deg(angle)
            print("Y: {}   X:    {}".format(self.waypoint_y-self.current_pos[1],self.waypoint_x-self.current_pos[0]))
            print("Angle: {}".format(angle))

#            if np.abs(angle) > 45.0:
#                # Turn in place
#                self.cmd_vel.linear.x = 0.0
#                self.cmd_vel.linear.y = 0.0
#                self.cmd_vel.angular.z = np.sign(angle)*MAX_ANG_VEL
#                old_vel_x = 0.0
#                old_vel_y = 0.0
#                old_ang_z = 0.0
#            else:
            self.cmd_vel.linear.x = vel_x
            self.cmd_vel.linear.y = vel_y
            self.cmd_vel.angular.z = vel_ang_z
            # If close to waypoint goal stop 
            print("Dist to waypoint: {}".format(np.linalg.norm(self.current_pos-np.array([self.waypoint_x,self.waypoint_y]))))
            if np.linalg.norm(self.current_pos-np.array([self.waypoint_x,self.waypoint_y])) < WAYPOINT_DIST_THRESHOLD:
                self.cmd_vel = Twist()
                old_vel_x = 0.0
                old_vel_y = 0.0
                old_ang_z = 0.0

            print(self.cmd_vel)
            self.cmd_vel_publisher.publish(self.cmd_vel)

            self.r.sleep()
	    #print(self.msg)


    def shutdown(self):
        rospy.loginfo("Path Follower Node is shutdown")
        rospy.sleep(1)

def main():
    try:
        path_follower = PathFollower()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()