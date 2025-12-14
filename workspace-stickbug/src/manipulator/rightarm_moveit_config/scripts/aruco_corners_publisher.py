import rospy
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
import numpy as np
import os
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import pyrealsense2 as rs

class ArucoDetectionNode:
    def __init__(self):
        rospy.init_node('aruco_corner_publisher')

        self.bridge = CvBridge()
        
        self.aruco_info_pub = rospy.Publisher('/aruco_corners', Float32MultiArray, queue_size=10)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters )

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

        if ids is not None:
            aruco_corners = Float32MultiArray()

            for i in range(len(ids)):
                marker_id = ids[i][0]
                marker_corners = corners[i][0]

                # Publish corner coordinates and depths
                for corner in marker_corners:
                    x, y = int(corner[0]), int(corner[1])
                    depth = depth_frame.get_distance(x, y)

                    # Append corner coordinates and depth to the array
                    aruco_corners.data.extend([x, y, depth])

            self.aruco_info_pub.publish(aruco_corners)

            # Draw the detected markers on the color image
            cv2.aruco.drawDetectedMarkers(color_image, corners)

        return color_image

    def run(self):
        rate = rospy.Rate(10) #hz
        while not rospy.is_shutdown():
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if color_frame and depth_frame:
                color_image = np.asanyarray(color_frame.get_data())
                color_image_with_aruco = self.detect_aruco(color_image, depth_frame)

            if self.window_name != None:
                cv2.imshow(self.window_name, color_image_with_aruco)
                cv2.waitKey(1)
            
            rate.sleep()

def main():
    aruco_detection_node = ArucoDetectionNode()
    aruco_detection_node.run()

if __name__ == '__main__':
    main()

