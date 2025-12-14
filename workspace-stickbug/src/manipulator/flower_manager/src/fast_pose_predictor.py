from pathlib import Path
from ultralytics import YOLO
import torch
import cv2
import numpy as np

#! Sunflower
from posenet import PoseResNet
from utils import *

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
        depth = depth.astype(np.float32)/1000 # converting depth to meters
        depth_val, depth_reliable, _ = get_depth_value(
            good_bb, depth, mask,
             near_plane = 0.1, far_plane = 2.5
        )

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
        if self.save_output:
            img_clone = rgb.copy()
            t = np.array([0,0,1])
            for R, bb in zip(rot_pred_np, good_bb):
                plot_axis_and_translate(img_clone, R, t, self.K, bb, self.height, self.width, 15)
            cv2.imwrite('flower_pose_plot.png', img_clone) 
            print(f"Output image saved to: flower_pose_plot.png")

        #! Combile rotations and translation
        Rt = np.repeat(np.eye(4)[None], rot_pred_np.shape[0], axis=0)
        Rt[:,:3,:3] = rot_pred_np
        Rt[:,:3,3] = xyz

        #! Return Flower Poses in Camer coordinate system
        return Rt