import torch
import roma
import yaml
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as sciR


def procrustes_to_rotmat(inp: torch.Tensor) -> torch.Tensor:
    """
    Works with a batch of procustes as well.
    """
    return roma.special_procrustes(inp.reshape(-1, 3, 3))


def squarify_bb(bb):
    xmin, ymin, xmax, ymax = bb
    xrange = xmax-xmin
    yrange = ymax-ymin
    diff = abs(xrange-yrange)
    if diff%2 == 0:
        decrease_min = diff/2
        increase_max = diff/2
    else:
        decrease_min = (diff+1)/2 # 
        increase_max = (diff-1)/2
    if xrange>yrange:
        ymin -= decrease_min
        ymax += increase_max
    elif xrange<yrange:
        xmin -= decrease_min
        xmax += increase_max
    
    final_bb = [int(xmin), int(ymin), int(xmax), int(ymax)]
    return final_bb


def bb_in_frame(bb, img_shape):
    h,w,_ = img_shape
    xmin, ymin, xmax, ymax = bb
    if xmin<0 or ymin<0 or xmax>w or ymax>h:
        return False
    else:
        return True 

 
def get_points3d(uv, Zray, K):
    """
    Args:
        uv (np.array): (N,2) Pixel cordinates
        Zray (np.array): (N,) Depth values in meters
        K : (3,3) Camera Intrinsics
    Returns:
        points3d (np.array): (N,3) 3D points in camera coordinate system
    """
    N = uv.shape[0]
    uv1 = np.hstack((uv, np.ones(N).reshape(-1,1)))
    xnyn1 = (np.linalg.inv(K)@uv1.T).T
    xnyn1_norm = np.linalg.norm(xnyn1, axis=1)
    Z = Zray/xnyn1_norm
    xyz = xnyn1*Z.reshape(-1,1)
    return xyz


def shrink_mask(mask, kernel_size):
    """Shrinks the contour of a binary mask using erosion.

    Args:
        mask: Binary mask as a 2D NumPy array (True/False or 0/1).
        kernel_size: Size of the structuring element (integer).
    Returns:
        Eroded binary mask.
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    eroded_mask = cv2.erode(mask.astype(np.uint8), kernel, iterations=1)
    eroded_mask = eroded_mask > 0 # Convert back to binary
    return eroded_mask


def get_depth_value(bbox, depth, seg_mask, scale = None, near_plane = 0.1, far_plane = 3.0, vis = False):
    """Get depth value
    
    Args: 
        bbox: Bounding box to extract depth data from depth image - xmin,ymin,xmax,ymax
        depth: Pixel value depth value in meters (normally loaded from .npy file)
        seg_mask: Segmenation mask Image 0=False, 255=True (normally read from uint8 grayscale image)
        scale: Depth scale factor
        near_plane: depth below this is bad
        far_plane: depth above this is bad
    Returns:
        Depth values in meters
        Depth Reliable Mask
        Depth Visualization or None
    """
    if scale: depth *= scale
    good_depth = np.logical_and(depth>near_plane,depth<far_plane) # near/far filter
    seg_mask = seg_mask>128 # Convert mask to boolean
    seg_mask = np.logical_and(seg_mask,good_depth)
    seg_mask = shrink_mask(seg_mask, 10)
    depth *= 1000 # depth to millimeters
    depth_values, depth_vis, depth_reliable = [], [], []
    for bb in bbox:
        wmin,hmin,wmax,hmax = bb
        depth_crop = depth[hmin:hmax, wmin:wmax]
        mask_crop = seg_mask[hmin:hmax, wmin:wmax]
        good_depths = depth_crop[mask_crop]
        # Less than 50 pixels of depth info is unreliable
        if good_depths.shape[0]<50:
            # print(good_depths.shape)
            depth_reliable.append(False)
        else:
            depth_reliable.append(True)
        # put depth=0 if no pixels found 
        if good_depths.shape[0]==0:
            depth_values.append(0)
        else:
            depth_values.append(np.mean(good_depths))
        if vis: 
            depth_vis_crop = np.where(mask_crop, depth_crop, 0)
            depth_vis_crop = cv2.resize(depth_vis_crop, (50,50))
            depth_vis.append(depth_vis_crop)
    depth_values =  np.array(depth_values)/1000 # Convert back to meters
    depth_reliable = np.array(depth_reliable)
    if vis: 
        depth_vis = np.array(depth_vis)
        return depth_values, depth_reliable, depth_vis
    else:
        return depth_values, depth_reliable, None
    
    
def read_intrinsics_yaml(filepath: str):
    with open(filepath, "r") as yaml_file:
        data = yaml.safe_load(yaml_file)
    return data


def read_intrinsics_yaml_to_K_h_w(filepath: str):
    data = read_intrinsics_yaml(filepath)
    return np.array([
        [data['fx'], 0, data['cx']],
        [0, data['fy'], data['cy']],
        [0,0,1]
    ]), data['h'], data['w']
    


def R2E(R):
    """Rotation matrix (3x3) to Euler angles (3,)"""
    return sciR.from_matrix(R).as_euler('zyx', degrees=True)  


def E2R(E):
    """Euler angles (3,) to Rotation matrix (3x3)"""
    return sciR.from_euler('zyx', E, degrees=True).as_matrix()


def nullify_yaw(Rmatrix):
    """
    Args:
        Rmatrix (np.ndarray): Rotation Matrix (3x3)
    Returns:
        np.ndarray: Yaw nullified rotation matrix (3x3)
    """
    euler_angles = R2E(Rmatrix)
    euler_angles[0] = 0.0 # Nullify Yaw rotation
    Rmatrix_yaw_nullified = E2R(euler_angles)
    return Rmatrix_yaw_nullified


def nullify_yaw_batch(rotmat):
    """
    Args:
        rotmat: rotation matrices
    Returns:
        Yaw nullified rotation matrices
    """
    euler_angles = R2E(rotmat)
    euler_angles[:,0] = 0.0 # Nullify Yaw rotation
    rotmat_yaw_nullified = E2R(euler_angles)
    return rotmat_yaw_nullified

def project_3d_to_2d(points, K, R, t):
    """
    Project set of 3d points to 2d image
    """
    t = t.reshape(-1,1)
    points_trans = R@points.T + t
    projection = K@points_trans
    projection /= projection[2]
    projection = projection[:2]
    return projection.T


def plot_axis_and_translate(image, R, t, K, bb, h, w, thickness=5):
    """
    Args:
        image (np.ndarray): Numpy Image, (H, W, 3)
        R: (3,3)
        t: (3,)
        K: (3,3)
        b: bounding box
        h: image height
        w: image width
        thickness (int): Thickness of Axis plot
    Returns:
        np.ndarray: Plotted Numpy Image, (H, W, 3)
    """
    xmin, ymin, xmax, ymax = bb
    # x_offset = xmin+(xmax-xmin)/2 - 2016
    # y_offset = ymin+(ymax-ymin)/2 - 1512
    
    x_offset = xmin+(xmax-xmin)/2 - w/2
    y_offset = ymin+(ymax-ymin)/2 - h/2
    
    points = np.array([
        [0,0,0], [1,0,0], [0,1,0], [0,0,1]
    ])*0.05
    
    
    points2d = project_3d_to_2d(points, K, R, t)
   
    points2d[:,0]  += x_offset
    points2d[:,1]  += y_offset
    
    # print(points2d.shape)
    # exit()
    points2d = points2d.astype(np.int32)
   
    cv2.line(image, points2d[0], points2d[1], color=(0,0,255), thickness=thickness) # X-axis
    cv2.line(image, points2d[0], points2d[2], color=(0,255,0), thickness=thickness) # Y-axis
    cv2.line(image, points2d[0], points2d[3], color=(255,0,0), thickness=thickness) # Z-axis
        
    return image
