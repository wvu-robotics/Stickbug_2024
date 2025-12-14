#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import TwistStamped

class CameraVelocityTransformer:
    def __init__(self):
        rospy.init_node('camera_velocity_transformer')

        # Initialize a TransformListener
        self.listener = tf.TransformListener()

        # Wait for the transformation to become available
        self.listener.waitForTransform("base","cam_link", rospy.Time(), rospy.Duration(4.0))

        # Create a publisher for the transformed velocity in the base frame
        self.pub_camvel_base = rospy.Publisher('camvel_base_rightarm', Twist, queue_size=10)

        # Subscribe to the camera velocity topic
        rospy.Subscriber("cmd_vel", Twist, self.camera_velocity_callback)
        

    def camera_velocity_callback(self, camera_velocity_msg):
        try:
            # Get the transformation (translation and rotation) from tool0 frame to base frame
            (trans, rot_q) = self.listener.lookupTransform("base", "cam_link", rospy.Time(0))

            # Create a homogeneous transformation matrix
            rot= tf.transformations.quaternion_matrix(rot_q)
            rotation=rot[:3, :3]
           
         
            # Extract camera linear and angular velocities from the message
            linear_velocity_tool0 = np.array([
                camera_velocity_msg.linear.x,
                camera_velocity_msg.linear.y,
                camera_velocity_msg.linear.z
            ])

            angular_velocity_tool0 = np.array([
                camera_velocity_msg.angular.x,
                camera_velocity_msg.angular.y,
                camera_velocity_msg.angular.z
            ])

            

            # Transform linear velocity
            linear_velocity_base = np.dot(rotation, linear_velocity_tool0)
            print("rotation matrix is ", rotation)
            print("linear velocity of tool is ", linear_velocity_tool0)
            print("linear velocity wrt base  is ", linear_velocity_base)
            # Transform angular velocity
            angular_velocity_base = np.dot(rotation, angular_velocity_tool0)

            # Publish the transformed velocities in the base frame
            camvel_base_msg = Twist()
            camvel_base_msg.linear.x = linear_velocity_base[0]
            camvel_base_msg.linear.y = linear_velocity_base[1]
            camvel_base_msg.linear.z = linear_velocity_base[2]
            camvel_base_msg.angular.x = angular_velocity_base[0]
            camvel_base_msg.angular.y = angular_velocity_base[1]
            camvel_base_msg.angular.z = angular_velocity_base[2]

            self.pub_camvel_base.publish(camvel_base_msg)
      
            
            print("the transformation is ",trans)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Transform lookup failed!")

if __name__ == '__main__':
    try:
       
        
        transformer = CameraVelocityTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

