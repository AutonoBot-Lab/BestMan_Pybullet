"""
@Description :   A few functions used in camera mounted on robot
@Author      :   Yan Ding 
@Time        :   2023/08/30 22:22:14
"""

import cv2
import pybullet as p
import pybullet_data
import math
import time
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import random
from matplotlib.colors import LinearSegmentedColormap
import sys
import os
from PIL import Image

"""
Get the utils module path
"""
# customized package
current_path = os.path.abspath(__file__)
utils_path = os.path.dirname(current_path)
if os.path.basename(utils_path) != "utils":
    raise ValueError('Not add the path of folder "utils", please check again!')
sys.path.append(utils_path)
from utils_PbVisualizer import PbVisualizer
from utils_PbClient import PbClient
from utils_PIDController import PIDController


class CameraInfo:
    # Initialization method for the CameraInfo class
    def __init__(
        self, pb_client, height=480, width=480):
        """
        Initialize a new CameraInfo object.

        Parameters:
            height (int): The height of the camera's image in pixels.
            width (int): The width of the camera's image in pixels.
        """

        self.pb_client = pb_client
        self.client_id = self.pb_client.get_client()
        
        # camera information
        self.height = height
        self.width = width

        print(f"CameraMessage(height={self.height}, width={self.width})")

    def get_image_from_camera(self, base_id, image_name, width=480*4, height=480*4, enable_save=False):
        # set camera position and target position
        base_position, orientation_quat = p.getBasePositionAndOrientation(base_id)
        camera_position = np.array(base_position) + np.array([0, 0, 2])
        rot_mat = np.array(p.getMatrixFromQuaternion(orientation_quat)).reshape(3, 3)
        local_forward_vec = np.array([math.pi * 2, 0, 0])
        forward_vec = rot_mat @ local_forward_vec
        target_position = np.array(base_position) + forward_vec
        view_mat = p.computeViewMatrix(
            cameraEyePosition=camera_position,
            cameraTargetPosition=target_position,
            cameraUpVector=[0, 0, 1],
        )
        proj_mat = p.computeProjectionMatrixFOV(
            fov=60.0,  # field of view
            aspect=0.8,  # scale, default=1
            nearVal=0.01,  # view distance min
            farVal=10,
        )
        # get image from the camera
        w, h, rgb, depth, seg = p.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view_mat,
            projectionMatrix=proj_mat,
            physicsClientId=self.client_id,
        )

        if enable_save:
            image = Image.fromarray(rgb)
            image.save(f'image/input/{image_name}.png')
            
        return rgb, depth, seg