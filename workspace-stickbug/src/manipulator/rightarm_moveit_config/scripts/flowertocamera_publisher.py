#! /usr/bin/env python3
import rospy
#from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge,CvBridgeError
import cv2
from ultralytics import YOLO

import numpy as np
import os
from geometry_msgs.msg import Point, Pose, PoseArray
from sensor_msgs.msg import Image


class YOLORealSenseTracker:
    def __init__(self):
        rospy.init_node('yolov8_realSense_tracker')
        # Constructing paths for .pt files using os.path.expanduser to expand the '~' to the user's home directory
        bramble_small_path = os.path.expanduser(f'~/workspace-stickbug/src/manipulator/rightarm_moveit_config/scripts/bramble_small.pt')
        bramble_classification_path = os.path.expanduser(f'~/workspace-stickbug/src/manipulator/rightarm_moveit_config/scripts/bramble_classification.pt')


        # Initialize YOLOv8 model
        self.model = YOLO(bramble_small_path)
        self.modelcal=YOLO(bramble_classification_path)

        self.color_frame = None
        self.depth_frame = None
        self.time_stamp = None
       
        # Create publishers
        self.flower_pub = rospy.Publisher('aruco_to_camera', PoseArray, queue_size=10)
        self.color_image_sub=rospy.Subscriber("realsense_color_image",Image,self.image_RGB_callback)
        self.depth_image_sub=rospy.Subscriber("realsense_depth_image",Image,self.image_depth_callback)

    def track_objects(self):
        

        try:
            while not rospy.is_shutdown():
                # Wait for frames from the RealSense camera
                
                color_frame = self.color_frame
                depth_frame = self.depth_frame

                if color_frame is not None and depth_frame is not None:
                    # Convert the RealSense color frame to a NumPy array
                    color_image = color_frame #np.asanyarray(color_frame.get_data())

                    # Perform YOLOv8 object tracking
                    results = self.model(color_image, show=False)
                    #result1 = self.modelcal(color_image,show=True)
                    flower_info = PoseArray()
                    flower_info.header.stamp = rospy.Time.now()
                    flower_info.header.frame_id = "stickbug/arm3/camera_frame"
                    for r in results:
                        for box in r.boxes:
                            # Extract id and bounding box coordinates 
                            id=box.id 
                            if id is not None:
                                id=box.id.int().tolist()
                            else:
                                pass

                            print('the id of box is ',id)

                            x1=int(box.xyxy[0][0])
                            x2=int(box.xyxy[0][2])
                            y1=int(box.xyxy[0][1])
                            y2=int(box.xyxy[0][3])
                            
                            x_center = int((x1+ x2) / 2)
                            y_center = int((y1 + y2) / 2)
                         
                            # Get depth value for the center
                            #depth_value = depth_frame.get_distance(x_center, y_center)
                            depth_value = depth_frame[y_center, x_center] * 0.0001  # Convert mm to meters

                          #get the detected image and pass to classifier
                            cropped_img = color_image[y1:y2, x1:x2]
                             
                            #cv2.imshow('cropped flower', cropped_img)
                            #cv2.waitKey(1000)
                            
                            results1 = self.modelcal(cropped_img,show=False)
                           
                            for r1 in results1:
                                index1 =r1.probs.top5[0]
                                index2 =r1.probs.top5[1]
                           
                                if r1.names[index1]=='visible-center':
                                    visible=True
                                    print("hey i am visible")
                                else:
                                    visible=False
                                    print("i am hidden")

                            # Convert pixel coordinates to camera coordinates
                            fx = 422  # Focal length in x direction
                            fy = 422  # Focal length in y direction
                            cx = 420  # Principal point x-coordinate
                            cy = 241  # Principal point y-coordinate
                            x_cam = (x_center - cx) * depth_value / fx
                            y_cam = (y_center - cy) * depth_value / fy
                             # Publish the center coordinate with depth value
                            
                            new_pose = Pose()
                            new_pose.position.x = x_cam  # x-coordinate of the center
                            new_pose.position.y = y_cam # y-coordinate of the center
                            new_pose.position.z = depth_value 
                            new_pose.orientation.x = 0
                            new_pose.orientation.y = 0
                            new_pose.orientation.z = 0
                            new_pose.orientation.w = visible
                            flower_info.poses.append(new_pose) # depth value at the center
                            #print("the confident score is", box.conf)
                            confidence_threshold=0.5
                            min=1e-6
                            
                            if box.conf>=confidence_threshold and depth_value>min:
                               # self.flower_pub.publish(new_pose)
                                self.flower_pub.publish(flower_info)
                                self.depth_frame = None
                                self.color_frame = None

                        

                # Break the loop when 'q' is pressed
                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #     break

        finally:
            pass
            #cv2.destroyAllWindows()

    def image_RGB_callback(self,ros_image):
        bridge = CvBridge()
        try:
            # Convert the ROS Image message to a CV2 image
            self.color_frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)



    def image_depth_callback(self,ros_image):
        bridge = CvBridge()
        try:
            # Convert the ROS Image message to a CV2 image
            #self.depth_frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
            self.depth_frame = bridge.imgmsg_to_cv2(ros_image, "passthrough")
            self.time_stamp = ros_image.header.stamp
        except CvBridgeError as e:
            rospy.logerr(e)
        

if __name__ == '__main__':
    yolo_realSense_tracker = YOLORealSenseTracker()
    yolo_realSense_tracker.track_objects()
