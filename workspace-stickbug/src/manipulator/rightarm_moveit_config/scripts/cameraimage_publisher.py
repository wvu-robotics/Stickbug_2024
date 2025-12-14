#! /usr/bin/env python3.9
import rospy
from sensor_msgs.msg import Image
import pyrealsense2 as rs
import numpy as np

pipeline = None

def realsense_publisher():
    global pipeline
    rospy.init_node('realsense_publisher', anonymous=True)
    color_image_pub = rospy.Publisher('realsense_color_image', Image, queue_size=10)
    depth_image_pub = rospy.Publisher('realsense_depth_image', Image, queue_size=10)

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    rate = rospy.Rate(10)  # Match the framerate of the camera
    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Prepare color image
        color_image = np.asanyarray(color_frame.get_data())
        ros_color_image = Image()
        ros_color_image.header.stamp = rospy.Time.now()
        ros_color_image.header.frame_id = "camera"
        ros_color_image.height, ros_color_image.width = color_image.shape[:2]
        ros_color_image.encoding = "bgr8"
        ros_color_image.is_bigendian = False
        ros_color_image.step = ros_color_image.width * 3  # 3 bytes per pixel for BGR8
        ros_color_image.data = color_image.tobytes()

        # Prepare depth image
        depth_image = np.asanyarray(depth_frame.get_data())
        ros_depth_image = Image()
        ros_depth_image.header.stamp = rospy.Time.now()
        ros_depth_image.header.frame_id = "camera"
        ros_depth_image.height, ros_depth_image.width = depth_image.shape
        ros_depth_image.encoding = "mono16"  # Assuming depth format is z16
        ros_depth_image.is_bigendian = False
        ros_depth_image.step = ros_depth_image.width * 2  # 2 bytes per pixel for mono16
        ros_depth_image.data = depth_image.tobytes()

        # Publish the images
        color_image_pub.publish(ros_color_image)
        depth_image_pub.publish(ros_depth_image)

        rate.sleep()

if __name__ == '__main__':
    try:
        realsense_publisher()
    except rospy.ROSInterruptException:
        pass
    finally:
        if pipeline:
            pipeline.stop()

	
