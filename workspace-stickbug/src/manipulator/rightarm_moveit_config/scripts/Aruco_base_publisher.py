#!/usr/bin/env python

import rospy
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose
import pyrealsense2 as rs
import tf

class ArucoDetectionNode:
    def __init__(self):
        rospy.init_node('aruco_detection_node')

        self.bridge = CvBridge()
        self.aruco_pub = rospy.Publisher('/aruco_markers', Float32MultiArray, queue_size=10)
        self.aruco_pose_pub = rospy.Publisher('/aruco_poses', Pose, queue_size=10)

        # Initialize TF listener
        self.listener = tf.TransformListener()

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()

        self.window_name = 'Aruco Detection'
        cv2.namedWindow(self.window_name)

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)

    def detect_aruco(self, color_image, depth_frame):
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            aruco_info = Float32MultiArray()

            for i in range(len(ids)):
                marker_corners = corners[i][0]

                # Calculate the center coordinates of the marker
                center_x = int((marker_corners[0][0] + marker_corners[2][0]) / 2)
                center_y = int((marker_corners[0][1] + marker_corners[2][1]) / 2)

                # Calculate the average depth value for the marker corners
                corner_depths = []
                for corner in marker_corners:
                    x, y = int(corner[0]), int(corner[1])
                    depth_value = depth_frame.get_distance(x, y)
                    corner_depths.append(depth_value)

                if len(corner_depths) > 0:
                    # Calculate the average depth
                    average_depth = np.mean(corner_depths)
                    fx = 422  # Focal length in x direction
                    fy = 422  # Focal length in y direction
                    cx = 420  # Principal point x-coordinate
                    cy = 241  # Principal point y-coordinate

                    # Convert pixel coordinates to camera coordinates
                    x_cam = (center_x - cx) * average_depth / fx
                    y_cam = (center_y - cy) * average_depth / fy

                    # Publish center coordinates and depth
                    aruco_info.data.append(x_cam)  # X-coordinate of the center
                    aruco_info.data.append(y_cam)  # Y-coordinate of the center
                    aruco_info.data.append(average_depth)  # Depth

                    # Draw the detected marker on the color image
                    cv2.aruco.drawDetectedMarkers(color_image, corners)

                    # Publish ArUco pose with respect to the base frame
                    try:
                        # Wait for the transformation
                        self.listener.waitForTransform("base", "cam_link", rospy.Time(), rospy.Duration(4.0))

                        # Transform the pose
                        (trans, rot) = self.listener.lookupTransform("base", "cam_link", rospy.Time(0))

                        # Construct the transformation matrix
                        T = tf.transformations.quaternion_matrix(rot)
                        T[:3, 3] = trans

                        # Define ArUco pose in camera frame
                        aruco_pose_cam = np.array([x_cam, y_cam, average_depth, 1])  # Homogeneous coordinates

                        # Apply the transformation
                        aruco_pose_base = np.dot(T, aruco_pose_cam)

                        # Create and publish the Pose message
                        aruco_pose_msg = Pose()
                        aruco_pose_msg.position.x = aruco_pose_base[0]
                        aruco_pose_msg.position.y = aruco_pose_base[1]
                        aruco_pose_msg.position.z = aruco_pose_base[2]
                        aruco_pose_msg.orientation.x = rot[0]
                        aruco_pose_msg.orientation.y = rot[1]
                        aruco_pose_msg.orientation.z = rot[2]
                        aruco_pose_msg.orientation.w = rot[3]
                        self.aruco_pose_pub.publish(aruco_pose_msg)

                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        rospy.logwarn("Failed to fetch transformation from cam_link to base_link.")

            self.aruco_pub.publish(aruco_info)

        return color_image

    def run(self):
        while not rospy.is_shutdown():
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if color_frame and depth_frame:
                color_image = np.asanyarray(color_frame.get_data())
                color_image_with_aruco = self.detect_aruco(color_image, depth_frame)

                cv2.imshow(self.window_name, color_image_with_aruco)
                cv2.waitKey(1)

def main():
    aruco_detection_node = ArucoDetectionNode()
    aruco_detection_node.run()

if __name__ == '__main__':
    main()
