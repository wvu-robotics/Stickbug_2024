#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
from geometry_msgs.msg import Point

# Initialize the ROS node
rospy.init_node('yolov8_realSense_tracker')

# Load the YOLOv8 model (Make sure 'best_bramble.pt' is in the correct path)
model = YOLO('best_bramble.pt')

# Initialize the RealSense camera pipeline
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Create a publisher for bounding box values
bbox_pub = rospy.Publisher('/bounding_boxes', Point, queue_size=10)

# Start the RealSense camera
pipe.start(cfg)

try:
    while not rospy.is_shutdown():
        # Wait for frames from the RealSense camera
        frames = pipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if color_frame and depth_frame:
            # Convert the RealSense color frame to a NumPy array
            color_image = np.asanyarray(color_frame.get_data())

            # Perform YOLOv8 object tracking
            #results = model.track(color_image, persist=True, show=True)
            results = model(color_image, show=True)

            for r in results:
                for box in r.boxes:
                    # Extract bounding box coordinates
                    x_center = int((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
                    y_center = int((box.xyxy[0][1] + box.xyxy[0][3]) / 2)

                    # Get depth value for the center
                    depth_value = depth_frame.get_distance(x_center, y_center)

                    # Publish bounding box values with depth as z value
                    bbox_msg = Point()
                    bbox_msg.x = x_center  # x-coordinate of the center
                    bbox_msg.y = y_center  # y-coordinate of the center
                    bbox_msg.z = depth_value  # depth value at the center
                    bbox_pub.publish(bbox_msg)

            # Display the camera feed
            cv2.imshow("Camera Feed", color_image)

            # Break the loop when 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

finally:
    pipe.stop()
    cv2.destroyAllWindows()

