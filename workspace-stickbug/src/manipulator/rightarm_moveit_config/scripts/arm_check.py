#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
# Import YOLOv8 dependencies here, adjust import as necessary
# from yolov8 import YOLOv8Model

def image_callback(ros_image):
    bridge = CvBridge()
    try:
        # Convert the ROS Image message to a CV2 image
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)

    # Process the CV2 image with YOLOv8 here
    # This is a placeholder, replace with actual YOLOv8 processing code
    # results = yolov8_model.process(cv_image)
    # For example, to simply display the image:
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

def main():
    rospy.init_node('image_processor', anonymous=True)
    rospy.Subscriber('realsense_color_image', Image, image_callback)

    # Prevent the script from exiting until the node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
	
