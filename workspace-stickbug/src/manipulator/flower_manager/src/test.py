import cv2
import numpy as np

from fast_pose_predictor import FastPosePredictor

#! Inputs paths
rgb_path = 'test_data/rgb.png'
depth_path = 'test_data/depth.png'
intirin_path = 'realsense405_intrinsics.yaml'
yolo_weights = 'yolo.pt'
posenet_weights = 'posenet.pt'
device = 'cuda'

#! Read inputs
rgb = cv2.imread(rgb_path)
depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

#! Initialize Fast Pose Predictor Model 
model = FastPosePredictor(
    device=device,
    yolo_path=yolo_weights,
    posenet_path=posenet_weights,
    intrin_path=intirin_path,
    save_output=True
)

#! Get flower poses
flower_pose = model.get_flower_poses(rgb, depth)
print(f"Flower pose extracted with shape: {flower_pose.shape}") # num flower x [4x4 rotation matrix]
