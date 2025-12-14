#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseArray
import tf

class YOLORealSenseTracker:
    def __init__(self):
        rospy.init_node('yolov8_realSense_tracker')

        # Initialize YOLOv8 model
        self.model = YOLO('best_bramble.pt')

        # Initialize RealSense camera pipeline
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Create publishers
        self.bbox_pub = rospy.Publisher('/bounding_boxes', Point, queue_size=10)
        self.pose_array_pub = rospy.Publisher('/camera_pose_array', PoseArray, queue_size=10)

        # Initialize TF2 buffer and listener
        #self.tf_buffer = tf2_ros.Buffer()
        #self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("base","cam_link", rospy.Time(), rospy.Duration(4.0))
    def track_objects(self):
        # Start the RealSense camera
        self.pipe.start(self.cfg)

        try:
            while not rospy.is_shutdown():
                # Wait for frames from the RealSense camera
                frames = self.pipe.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()

                if color_frame and depth_frame:
                    # Convert the RealSense color frame to a NumPy array
                    color_image = np.asanyarray(color_frame.get_data())

                    # Perform YOLOv8 object tracking
                    results = self.model(color_image, show=True)

                    for r in results:
                        for box in r.boxes:
                            # Extract bounding box coordinates
                            x_center = int((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
                            y_center = int((box.xyxy[0][1] + box.xyxy[0][3]) / 2)

                            # Get depth value for the center
                            depth_value = depth_frame.get_distance(x_center, y_center)
                            
                            #convert to camera coorodiate 
                            
                            # Assume you have access to the camera's intrinsic parameters
                            fx = 422  # Focal length in x direction
                            fy = 422  # Focal length in y direction
                            cx = 420  # Principal point x-coordinate
                            cy = 241 # Principal point y-coordinate

                            # Convert pixel coordinates to camera coordinates
                            x_cam = (x_center - cx) * depth_value / fx
                            y_cam = (y_center - cy) * depth_value / fy


                            # Publish bounding box values with depth as z value
                            bbox_msg = Point()
                            bbox_msg.x = x_cam  # x-coordinate of the center
                            bbox_msg.y = y_cam # y-coordinate of the center
                            bbox_msg.z = depth_value  # depth value at the center
                            
                            self.bbox_pub.publish(bbox_msg)

                            # Publish camera pose with respect to base link
                            pose_array_msg = PoseArray()
                            pose_array_msg.header.stamp = rospy.Time.now()

                            # Fetch transformation from base to cam_link
                            try:
                                (trans, rot) = self.listener.lookupTransform("base", "cam_link", rospy.Time(0))
                                pose = Pose()
                                pose.position.x = trans[0]
                                pose.position.y = trans[1]
                                pose.position.z = trans[2]
                                pose.orientation.x = rot[0]
                                pose.orientation.y = rot[1]
                                pose.orientation.z = rot[2]
                                pose.orientation.w = rot[3]
                                pose_array_msg.poses.append(pose)
                            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                                rospy.logwarn("Failed to fetch transformation from base to cam_link.")

                            # Publish the PoseArray message
                            self.pose_array_pub.publish(pose_array_msg)

                # Break the loop when 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.pipe.stop()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    yolo_realSense_tracker = YOLORealSenseTracker()
    yolo_realSense_tracker.track_objects()

