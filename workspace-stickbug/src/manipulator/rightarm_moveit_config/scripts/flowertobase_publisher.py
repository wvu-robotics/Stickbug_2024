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
        self.pose_array_pub = rospy.Publisher('/flower_poses', PoseArray, queue_size=10)

        # Initialize TF listener
        self.listener = tf.TransformListener()

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

                    flower_poses = PoseArray()
                    flower_poses.header.stamp = rospy.Time.now()
                    flower_poses.header.frame_id = "base"  # Adjust the frame ID as needed

                    for r in results:
                        for box in r.boxes:
                            # Extract bounding box coordinates
                            x_center = int((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
                            y_center = int((box.xyxy[0][1] + box.xyxy[0][3]) / 2)

                            # Get depth value for the center
                            depth_value = depth_frame.get_distance(x_center, y_center)

                            # Convert pixel coordinates to camera coordinates
                            fx = 422  # Focal length in x direction
                            fy = 422  # Focal length in y direction
                            cx = 420  # Principal point x-coordinate
                            cy = 241  # Principal point y-coordinate
                            x_cam = (x_center - cx) * depth_value / fx
                            y_cam = (y_center - cy) * depth_value / fy
                             # Publish bounding box values with depth as z value
                            bbox_msg = Point()
                            bbox_msg.x = x_cam  # x-coordinate of the center
                            bbox_msg.y = y_cam # y-coordinate of the center
                            bbox_msg.z = depth_value  # depth value at the center
                            
                            self.bbox_pub.publish(bbox_msg)

                            # Transform flower point from camera frame to base frame
                            try:
                                # Wait for the transformation
                                self.listener.waitForTransform("base", "cam_link", rospy.Time(), rospy.Duration(4.0))

                                # Transform the point
                                (trans, rot) = self.listener.lookupTransform("base", "cam_link", rospy.Time(0))
                                trans = np.array(trans)
                                rotation = np.array(rot)
                                flower_pos = np.array([x_cam, y_cam, depth_value])
                                 # Append 1 to make the flower_pos homogeneous
                                flower_pos_homo = np.append(flower_pos, 1)

                                 # Construct the transformation matrix
                                T = tf.transformations.quaternion_matrix(rot)
                                T[:3, 3] = trans

                                 # Apply the transformation
                                transformed_flower_pos_homo = np.dot(T, flower_pos_homo)

                                 # Extract the transformed position
                                transformed_flower_pos = transformed_flower_pos_homo[:3]


                                # Create and append the Pose message
                                pose = Pose()
                                pose.position.x = transformed_flower_pos[0]
                                pose.position.y = transformed_flower_pos[1]
                                pose.position.z = transformed_flower_pos[2]
                                pose.orientation.x = rot[0]
                                pose.orientation.y = rot[1]
                                pose.orientation.z = rot[2]
                                pose.orientation.w = rot[3]
                                flower_poses.poses.append(pose)

                            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                                rospy.logwarn("Failed to fetch transformation from cam_link to base_link.")

                    # Publish the PoseArray message
                    self.pose_array_pub.publish(flower_poses)

                # Break the loop when 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.pipe.stop()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    yolo_realSense_tracker = YOLORealSenseTracker()
    yolo_realSense_tracker.track_objects()
