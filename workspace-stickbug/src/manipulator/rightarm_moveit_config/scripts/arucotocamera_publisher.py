#!/usr/bin/env python3.9

import rospy
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose
import os 
import pyrealsense2 as rs


class ArucoDetectionNode:
    def __init__(self):
        rospy.init_node('aruco_detection_node')

        self.bridge = CvBridge()
        self.aruco_pub = rospy.Publisher('aruco_to_camera', PoseArray, queue_size=10)
        self.aruco_cornor_pub = rospy.Publisher('aruco_corners', Float32MultiArray, queue_size=10)
        

        #self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        #self.parameters = aruco.DetectorParameters_create()
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters =  cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        if 'DISPLAY' in os.environ:
            self.window_name = 'Aruco Detection'
            cv2.namedWindow(self.window_name)
        else:
            self.window_name = None
            self.ssh = False
            self.ssh = True

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)

    def detect_aruco(self, color_image, depth_frame):
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        aruco_corners=Float32MultiArray()
        aruco_info = PoseArray()
        aruco_info.header.stamp = rospy.Time.now()
        aruco_info.header.frame_id = "camera_frame"

        if ids is not None:

            for i in range(len(ids)):
                marker_corners = corners[i][0]

                for corner in marker_corners:
                        x, y = int(corner[0]), int(corner[1])
                        depth = depth_frame.get_distance(x, y)

                        # Append corner coordinates and depth to the array
                        aruco_corners.data.extend([x, y, depth])
                

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
                    new_pose = Pose()
                    new_pose.position.x = x_cam
                    new_pose.position.y = y_cam
                    new_pose.position.z = average_depth
                    new_pose.orientation.x = 0
                    new_pose.orientation.y = 0
                    new_pose.orientation.z = 0
                    new_pose.orientation.w = 1

                    aruco_info.poses.append(new_pose)

                    #aruco_info.data.append(x_cam)  # X-coordinate of the center
                    #aruco_info.data.append(y_cam)  # Y-coordinate of the center
                    #aruco_info.data.append(average_depth)  # Depth

                    # Draw the detected marker on the color image
                    cv2.aruco.drawDetectedMarkers(color_image, corners)
        
        self.aruco_pub.publish(aruco_info)
        self.aruco_cornor_pub.publish(aruco_corners)
         
        return color_image

    def run(self):
        while not rospy.is_shutdown():
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if color_frame and depth_frame:
                color_image = np.asanyarray(color_frame.get_data())
                color_image_with_aruco = self.detect_aruco(color_image, depth_frame)
                cv2.rectangle(color_image_with_aruco,(200,290), (370,440), color=(0,255,0), thickness=5)

            if self.window_name != None:
                cv2.imshow(self.window_name, color_image_with_aruco)
                cv2.waitKey(1)

def main():
    aruco_detection_node = ArucoDetectionNode()
    aruco_detection_node.run()
   

if __name__ == '__main__':
    main()
