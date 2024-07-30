import cv2
import numpy as np
import pybullet as p
from PIL import Image
from datetime import datetime
import matplotlib.pyplot as plt
from typing import Type, Tuple
from dataclasses import dataclass
from matplotlib.colors import LinearSegmentedColormap

@dataclass
class CameraParameters:
    """Used to store the camera's intrinsic parameters, head tilt angle, image data, color data, and depth data.
    """
    
    fx: float                       # The focal length scaling factor in the x direction. It is typically the focal length in pixel units and is used to describe the camera's intrinsic parameters.
    fy: float                       # The focal length scaling factor in the y direction. Similar to fx, but for the y direction.
    cx: float                       # The x-coordinate of the camera's principal point (optical center). It represents the position of the optical axis's intersection on the image plane in the x direction, usually near the center of the image.
    cy: float                       # The y-coordinate of the camera's principal point (optical center). Similar to cx, but for the y direction.
    head_tilt: float                # The tilt angle of the camera head, describing the camera's rotation or inclination.
    image: Type[Image.Image]        # An image object from the PIL library, representing the captured image data.
    colors: np.ndarray              # A NumPy array containing color data of the image, typically used for processing color information within the image.
    depths: np.ndarray              # A NumPy array containing depth data of the image, representing the distance of each pixel point from the camera.


class Camera:
    """Robotic Camera.
    
    This class handles the camera functionalities for a robotic system, including capturing RGB and depth images.
    """
    
    def __init__(self, cfg, base_id, arm_height):
        """
        Initializes the Camera class with configuration, base ID, and arm height.

        Args:
            cfg (object): Configuration settings for the camera.
            base_id (int): The base ID of the robot.
            arm_height (float): The height of the robot arm.
        """
        self.base_id = base_id
        self.arm_height = arm_height
        
        self.width = cfg.width      # W
        self.height = cfg.height    # H
        self.nearVal = cfg.nearVal
        self.farVal = cfg.farVal
        # self.fx = self.fx,
        # self.fy = cfg.fy,
        # self.cx = cfg.cx,
        # self.cy = cfg.cy,
        # self.head_tilt = cfg.head_tilt,
        
        # get rgb and depth image
        _, _, rgb, depth, _ = self.update()
        self.image = Image.fromarray(rgb)
        self.colors = np.array(rgb)
        self.depths = np.array(depth)

    
    def update(self):
        """
        Updates the camera view and projection matrices, captures images, and returns the captured data.
        
        Returns:
            tuple: Width, height, RGB image, depth image, and segmentation mask.
        """
        # get base pose
        position, orientation = p.getBasePositionAndOrientation(self.base_id)
        camera_position = np.array([position[0], position[1], self.arm_height + 0.3])
        
        # The three direction vectors of the camera in the world coordinate system
        r_mat = p.getMatrixFromQuaternion(orientation)
        tx_vec = np.array([r_mat[0], r_mat[3], r_mat[6]])   # x direction vector, the right side of the camera is facing
        ty_vec = np.array([r_mat[1], r_mat[4], r_mat[7]])   # y direction vector, the camera's forward direction
        tz_vec = np.array([r_mat[2], r_mat[5], r_mat[8]])   # z direction vector, the vertical direction of the camera
        
        # set camera looking forward
        # target_position = camera_position - 1 * tx_vec
        target_position = camera_position + 1 * tx_vec

        view_mat = p.computeViewMatrix(
                        cameraEyePosition=camera_position,
                        cameraTargetPosition=target_position,
                        cameraUpVector=tz_vec
                    )

        # A projection matrix based on the field of view (FOV) to simulate the camera's perspective
        proj_mat = p.computeProjectionMatrixFOV(
                        fov=60,                            # 摄像头的视线夹角
                        aspect=self.width/self.height,     # 图像的宽高比
                        nearVal=self.nearVal,              # 摄像头视距min
                        farVal=self.farVal                 # 摄像头视距max
                    )
        
        # width, height, rgb, image, seg
        w, h, rgb, depth, seg = p.getCameraImage(
                                    width=self.width,
                                    height=self.height,
                                    viewMatrix=view_mat,
                                    projectionMatrix=proj_mat,
                                    shadow=0,     # Disable Shadows
                                    renderer=p.ER_TINY_RENDERER
                                )

        return w, h, rgb, depth, seg
    
    
    def get_rgb_image(self, enable_show=False, enable_save=False):
        """
        Captures an RGB image from the camera.

        Args:
            enable_show (bool, optional): Whether to display the captured image. Defaults to False.
            enable_save (bool, optional): Whether to save the captured image. Defaults to False.

        Returns:
            np.ndarray: Captured RGB image.
        """
        
        # get base pose
        position, orientation = p.getBasePositionAndOrientation(self.base_id)
        camera_position = np.array([position[0], position[1], self.arm_height + 0.3])
        
        # The three direction vectors of the camera in the world coordinate system
        r_mat = p.getMatrixFromQuaternion(orientation)
        tx_vec = np.array([r_mat[0], r_mat[3], r_mat[6]])   # x direction vector, the right side of the camera is facing
        ty_vec = np.array([r_mat[1], r_mat[4], r_mat[7]])   # y direction vector, the camera's forward direction
        tz_vec = np.array([r_mat[2], r_mat[5], r_mat[8]])   # z direction vector, the vertical direction of the camera
        
        # set camera looking forward
        # target_position = camera_position - 1 * tx_vec
        target_position = camera_position + 1 * tx_vec

        view_mat = p.computeViewMatrix(
                        cameraEyePosition=camera_position,
                        cameraTargetPosition=target_position,
                        cameraUpVector=tz_vec
                    )

        # A projection matrix based on the field of view (FOV) to simulate the camera's perspective
        proj_mat = p.computeProjectionMatrixFOV(
                        fov=60,       # 摄像头的视线夹角
                        aspect=self.width/self.height,     # 图像的宽高比
                        nearVal=self.nearVal,   # 摄像头视距min
                        farVal=self.farVal       # 摄像头视距max
                    )
        
        # width, height, rgb, image, seg
        _, _, rgb, _, _ = p.getCameraImage(
                                width=self.width,
                                height=self.height,
                                viewMatrix=view_mat,
                                projectionMatrix=proj_mat,
                                shadow=0     # Disable Shadows
                            )

        if enable_show:
            plt.imshow(rgb)
            plt.axis('off')
            plt.title('rgb image')
            plt.show()

        if enable_save:
            current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
            rgb_path = f"../Examples/image/rgb_{current_time}.png"
            rgbImg = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
            cv2.imwrite(rgb_path, rgbImg)
            # Image.fromarray(rgb).save(rgb_path)
        
        return rgb
    
    
    def get_depth_image(self, enable_show=False, enable_save=False):
        """
        Captures a depth image from the camera.

        Args:
            enable_show (bool, optional): Whether to display the captured image. Defaults to False.
            enable_save (bool, optional): Whether to save the captured image. Defaults to False.

        Returns:
            np.ndarray: Captured depth image.
        """
        
        # get base pose
        position, orientation = p.getBasePositionAndOrientation(self.base_id)
        camera_position = np.array([position[0], position[1], self.arm_height + 0.3])
        
        # The three direction vectors of the camera in the world coordinate system
        r_mat = p.getMatrixFromQuaternion(orientation)
        tx_vec = np.array([r_mat[0], r_mat[3], r_mat[6]])   # x direction vector, the right side of the camera is facing
        ty_vec = np.array([r_mat[1], r_mat[4], r_mat[7]])   # y direction vector, the camera's forward direction
        tz_vec = np.array([r_mat[2], r_mat[5], r_mat[8]])   # z direction vector, the vertical direction of the camera
        
        # set camera looking forward
        # target_position = camera_position - 1 * tx_vec
        target_position = camera_position + 1 * tx_vec

        view_mat = p.computeViewMatrix(
                        cameraEyePosition=camera_position,
                        cameraTargetPosition=target_position,
                        cameraUpVector=tz_vec
                    )

        # A projection matrix based on the field of view (FOV) to simulate the camera's perspective
        proj_mat = p.computeProjectionMatrixFOV(
                        fov=60,       # 摄像头的视线夹角
                        aspect=self.width/self.height,     # 图像的宽高比
                        nearVal=self.nearVal,   # 摄像头视距min
                        farVal=self.farVal       # 摄像头视距max
                    )
        
        # width, height, rgb, image, seg
        _, _, _, depth, _ = p.getCameraImage(
                                width=self.width,
                                height=self.height,
                                viewMatrix=view_mat,
                                projectionMatrix=proj_mat,
                                shadow=0     # Disable Shadows
                            )

        # Convert normalized depth values ​​to actual depth values
        depth = self.farVal * self.nearVal / (self.farVal - (self.farVal - self.nearVal) * depth)

        # image_point = [320, 240]
        # crop_size = 200
        # depth_image = self.crop_image(depth, image_point, crop_size)

        if enable_show:
            # Linear color mapping from black (the color when the depth value is 0) to white (the color when the depth value is 1)
            cdict = {
                "red": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
                "green": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
                "blue": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
            }
            custom_cmap = LinearSegmentedColormap("custom_cmap", cdict)
            plt.imshow(depth, cmap=custom_cmap)
            plt.colorbar()
            plt.title('depth image')
            plt.show()

        if enable_save:
            current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
            depth_path = f"../Examples/image/depth_{current_time}.png"
            # plt.imsave(depth_path, depth, cmap=custom_cmap)
            depth_img = (depth * 1000).astype(np.uint16)
            Image.fromarray(depth_img).save(depth_path)
        
        return depth