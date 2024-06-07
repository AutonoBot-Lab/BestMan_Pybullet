import numpy as np
from Grasp_Pose_Estimation import CameraParameters

def get_3d_points(cam: CameraParameters):
    """Convert depth image to 3D point cloud

    Args:
        cam (CameraParameters): Camera internal parameters
    
    Returns:
        points(np.array): 3D point cloud
    """
    
    xmap, ymap = np.arange(cam.depths.shape[1]), np.arange(cam.depths.shape[0])
    xmap, ymap = np.meshgrid(xmap, ymap)
    points_z = cam.depths
    points_x = (xmap - cam.cx) / cam.fx * points_z
    points_y = (ymap - cam.cy) / cam.fy * points_z
    points = np.stack((points_x, points_y, points_z), axis=2)
    return points