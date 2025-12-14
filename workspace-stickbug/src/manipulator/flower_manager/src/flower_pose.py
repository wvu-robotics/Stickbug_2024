#! /usr/bin/env python3

import sys
print(f"Python {sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}")

import rospy
import cv2
from cv_bridge import CvBridge,CvBridgeError
import numpy as np
import os
from geometry_msgs.msg import Point, Pose, PoseArray
from sensor_msgs.msg import Image
import tf.transformations as tf_trans  # Import for quaternion conversion

from pathlib import Path
from ultralytics import YOLO
import torch

#! Sunflower
from posenet import PoseResNet
from utils import *

import tf.transformations as tf_trans

class FastPosePredictor():
    def __init__(self,
        device: str,
        yolo_path: str,
        posenet_path: str,
        intrin_path: str,
        save_output: bool = False
    ):
        self.device = device
        self.save_output = save_output
    
        #! Posenet
        self.posenet = PoseResNet().to(device)
        self.posenet.load_state_dict(torch.load(posenet_path, weights_only=True))
        print(f"Model loaded: {Path(posenet_path).name}")
        
        #! Load YOLO
        self.yolo = YOLO(yolo_path).to(device)
        print(f"YOLO loaded: {Path(yolo_path).name}")
        
        #! Intrinsics
        self.K, self.height, self.width = read_intrinsics_yaml_to_K_h_w(intrin_path)
        
        print("FastPosePredictor initialized!")
       
 
    def get_bbox_mask(self, image):
        """
        Returns flower detection bounding boxes and segmentation mask
        """
        bbox = None
        mask = None
        H,W,_ = image.shape
        results = self.yolo(image)
        if results[0].masks is not None:
            masks = results[0].masks.data
            mask = torch.sum(masks, axis=0)
            mask = torch.clip(mask, 0, 1)*255
            mask = mask.cpu().numpy().astype(np.uint8)
            mask = cv2.resize(mask, (W,H))
            bbox = results[0].boxes.xyxy  # Bounding boxes in [x1, y1, x2, y2] format
            bbox = bbox.cpu().numpy().astype(np.int16)
        return bbox, mask
        

    def get_flower_poses(self, rgb, depth):
        bb_dino, mask = self.get_bbox_mask(rgb)

        #! Squarify bb and filter 
        sq_bb = []
        uv = []
        good_bb = []
        Rt = None
        if bb_dino is not None and mask is not None:
            for bb in bb_dino:
                xmin, ymin, xmax, ymax = bb
                u = (xmax+xmin)/2
                v = (ymax+ymin)/2
                sbb = squarify_bb(bb)
                if not bb_in_frame(sbb, rgb.shape):
                    continue
                uv.append([u,v])
                sq_bb.append(sbb)
                good_bb.append(bb)
            uv = np.array(uv)
            good_bb = np.array(good_bb).astype(np.int16)
            sq_bb = np.array(sq_bb)
        
            # Return None if no good bb 
            if good_bb.shape[0] == 0:
                return None

            #! Get Depth Values
            print(f'max depth {np.max(depth)} and min depth{np.min(depth)}')
            depth = depth.astype(np.float32)/10000 # converting depth to meters
            print(f'max depth {np.max(depth)} and min depth{np.min(depth)}')
            depth_val, depth_reliable, _ = get_depth_value(
                good_bb, depth, mask,
                near_plane = 0.1, far_plane = 2.5
            )
            print(f'depth value {depth_val}')

            #! Filter out unreliable depth values
            depth_val = depth_val[depth_reliable]
            uv = uv[depth_reliable]
            sq_bb = sq_bb[depth_reliable]
            
            if sq_bb.shape[0] == 0:
                return None

            #! Lift 2d points to 3d using depth
            xyz = get_points3d(uv, depth_val, self.K)

            #! Create batch of flower crops
            image_batch_np = [] 
            for bb in sq_bb: 
                xmin, ymin, xmax, ymax = bb
                
                img_crop = rgb[ymin:ymax, xmin:xmax]
                mask_crop = mask[ymin:ymax, xmin:xmax]
                        
                img_crop_sized = cv2.resize(img_crop, (512,512), interpolation=cv2.INTER_LANCZOS4)
                mask_crop_sized = cv2.resize(mask_crop, (512,512), interpolation=cv2.INTER_LANCZOS4)
                
                img_crop_sized_nobg = img_crop_sized * (mask_crop_sized.reshape(512,512,1)/255.0)
                image_batch_np.append(img_crop_sized_nobg)
                
            image_batch_np = np.array(image_batch_np)/255.0
            image_batch = torch.as_tensor(image_batch_np, dtype=torch.float32)
            image_batch = torch.permute(image_batch, (0,3,1,2)).to(self.device)

            #! Use PoseNet to get Rotation Matrices
            r9_M_pred = self.posenet(image_batch) 
            rot_pred = procrustes_to_rotmat(r9_M_pred)
            rot_pred_np = rot_pred.detach().cpu().numpy()  # (B,3,3)
            
            #! Nullify Yaw
            rot_pred_np = nullify_yaw_batch(rot_pred_np)

            #! Plot axis quick
            # if self.save_output:
            img_clone = rgb.copy()
            t = np.array([0,0,1])
            for R, bb in zip(rot_pred_np, good_bb):
                plot_axis_and_translate(img_clone, R, t, self.K, bb, self.height, self.width, 15)

            success = cv2.imwrite('hehe.png', img_clone)

            if not success:
                print("Failed to save image!")
 
            print(f"Output image saved to: flower_pose_plot.png")

            #! Combile rotations and translation
            Rt = np.repeat(np.eye(4)[None], rot_pred_np.shape[0], axis=0)
            Rt[:,:3,:3] = rot_pred_np
            Rt[:,:3,3] = xyz

        #! Return Flower Poses in Camer coordinate system
        return Rt



class FlowerPoseTracker:
    def __init__(self):
        rospy.init_node('flower_pose_tracker')
        # Constructing paths for .pt files using os.path.expanduser to expand the '~' to the user's home directory
        
        intirin_path = os.path.expanduser(f'~/workspace-stickbug/src/manipulator/flower_manager/src/realsense405_intrinsics.yaml')
        yolo_weights = os.path.expanduser(f'~/workspace-stickbug/src/manipulator/flower_manager/src/yolo.pt')
        posenet_weights = os.path.expanduser(f'~/workspace-stickbug/src/manipulator/flower_manager/src/posenet_for_sam.pth')
        device = 'cuda'

        #! Initialize Fast Pose Predictor Model 
        self.model = FastPosePredictor(
            device=device,
            yolo_path=yolo_weights,
            posenet_path=posenet_weights,
            intrin_path=intirin_path,
            save_output=True
        )

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
                    

                    #! Get flower poses
                    flowers = self.model.get_flower_poses(color_frame, depth_frame) #  flower x [4x4 rotation matrix]
                   
                    if flowers is not None:
                        flower_info = PoseArray()
                        flower_info.header.stamp = rospy.Time.now()
                        flower_info.header.frame_id = "stickbug/arm3/camera_frame"
                        
                        # Define the rotation from Z-axis to X-axis (90-degree rotation around Y-axis)
                        z_to_x_quaternion = tf_trans.quaternion_from_euler(0, -np.pi/2, 0)  # (roll=0, pitch=90°, yaw=0)

                        
                        for f in flowers:
                            # get position
                            flower_pos_x = f[0,3]
                            flower_pos_y = f[1,3]
                            flower_pos_z = f[2,3]
                            
                            # get orientation
                            quaternion = tf_trans.quaternion_from_matrix(f)
                            # correct orientation to be about the x axis
                            quaternion = tf_trans.quaternion_multiply(z_to_x_quaternion, quaternion)
                            
                            
                            new_pose = Pose()
                            new_pose.position.x = flower_pos_x  
                            new_pose.position.y = flower_pos_y 
                            new_pose.position.z = flower_pos_z 
                            new_pose.orientation.x = quaternion[0]
                            new_pose.orientation.y = quaternion[1]
                            new_pose.orientation.z = quaternion[2]
                            new_pose.orientation.w = quaternion[3]
                            flower_info.poses.append(new_pose) 
                                
                                                        
                        self.flower_pub.publish(flower_info)
                        self.depth_frame = None
                        self.color_frame = None

        finally:
            pass
         

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
    flower_pose_tracker = FlowerPoseTracker()
    flower_pose_tracker.track_objects()
    
    
    
    
    
