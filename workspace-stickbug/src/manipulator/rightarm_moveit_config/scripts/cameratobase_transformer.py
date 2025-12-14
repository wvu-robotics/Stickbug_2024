#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import tf



class ArucoDetectionNode:

    def __init__(self):
        rospy.init_node('aruco_transform_node')
        self.common_frame = 'base'
        namespace = rospy.get_namespace()  # Get the current namespace
        

        #------------------------ get common frame parameter ----------------
        # Construct the parameter name dynamically
        param_name = namespace + "common_frame"  # Assuming 'u2d2' is the parameter you want to access
        # Get a parameter, including its namespace, with a default value if the parameter is not found
        param_val = rospy.get_param(param_name, self.common_frame)
        if param_val  == self.common_frame:
            rospy.logwarn(f"The default value for 'common_frame' is being used: {self.common_frame}")
        self.common_frame = param_val
        print('Parameter value:',  self.common_frame )
        #-------------------------------------------------------------------------

        self.ns = namespace[1:]
       
        self.aruco_pose_pub = rospy.Publisher('raw_flowers', PoseArray, queue_size=10) # aruco_poses

        # Initialize TF listener
        self.listener = tf.TransformListener()

        #to_camera'
        rospy.Subscriber('aruco_to_camera', PoseArray, self.transform_callback)


    def transform_callback(self, data):
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = self.common_frame
        
        # Use the timestamp from the data's header
        data_timestamp = data.header.stamp

        # Iterate over the poses in the PoseArray
        for pose_msg in data.poses:
            if pose_msg.orientation.w == 0:
                visible = False
                pose_msg.orientation.w = 1.0
            else:
                visible = True
                    
            try:
                # Wait for the transformation, using the timestamp from data's header
                self.listener.waitForTransform(self.common_frame, self.ns + "camera_frame", data_timestamp, rospy.Duration(4.0))
                # Lookup the transformation using the timestamp from data's header
                (trans, rot) = self.listener.lookupTransform(self.common_frame, self.ns + "camera_frame", data_timestamp)
                (trans_end, rot_end) = self.listener.lookupTransform(self.common_frame, self.ns + "end_link", data_timestamp)

                # Construct the transformation matrix from camera to common frame
                T = tf.transformations.quaternion_matrix(rot)
                T[:3, 3] = trans

                # Extract pose position and convert to homogeneous coordinates
                aruco_pose_cam = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z, 1])

                # Apply the transformation
                aruco_pose_base = np.dot(T, aruco_pose_cam)

                # Update pose_msg with transformed position
                pose_msg.position.x = aruco_pose_base[0]
                pose_msg.position.y = aruco_pose_base[1]
                pose_msg.position.z = aruco_pose_base[2]

                # Assuming orientation remains the same or is updated elsewhere
                # pose_msg.orientation is set directly from rot_end if required
                if visible:
                    pose_msg.orientation.x = rot_end[0]
                    pose_msg.orientation.y = rot_end[1]
                    pose_msg.orientation.z = rot_end[2]
                    pose_msg.orientation.w = rot_end[3]
                else:
                    pose_msg.orientation.x = 0
                    pose_msg.orientation.y = 0
                    pose_msg.orientation.z = 0
                    pose_msg.orientation.w = 0
                    

                pose_array.poses.append(pose_msg)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to fetch transformation.")

        # Publish the transformed PoseArray
        self.aruco_pose_pub.publish(pose_array)

    
def main():
    aruco_detection_node = ArucoDetectionNode()
    rospy.spin()

if __name__ == '__main__':
    main()
