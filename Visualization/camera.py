import numpy as np
import pybullet as p
from PIL import Image
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
    """Robotic Camera
    """
    
    def __init__(self):

    
    def set_camera(self, base_id, width: int = 224, height: int = 224):
        
        position, orientation = p.getBasePositionAndOrientation(base_id, physicsClientId=self.client_id)
        print('pose', position, orientation)
        
        r_mat = p.getMatrixFromQuaternion(orientation)
        tx_vec = np.array([r_mat[0], r_mat[3], r_mat[6]])
        ty_vec = np.array([r_mat[1], r_mat[4], r_mat[7]])
        tz_vec = np.array([r_mat[2], r_mat[5], r_mat[8]])
        camera_position = np.array(position)
        target_position = camera_position - 1 * ty_vec

        view_mat = p.computeViewMatrix(
                        cameraEyePosition=camera_position,
                        cameraTargetPosition=target_position,
                        cameraUpVector=tz_vec
                    )

        proj_mat = p.computeProjectionMatrixFOV(
                        fov=60.0,  # 摄像头的视线夹角
                        aspect=1.0,
                        nearVal=0.01,  # 摄像头视距min
                        farVal=10  # 摄像头视距max
                    )
        
        w, h, rgb, depth, seg = p.getCameraImage(
                                    width=width,
                                    height=height,
                                    viewMatrix=view_mat,
                                    projectionMatrix=proj_mat,
                                    physicsClientId=self.client_id
                                )

        return w, h, rgb, depth, seg
    

    def get_depth_image(
        self, basePos, cameraPos, cameraUp, enable_show=False, enable_save=False
    ):
        """
        Capture a cropped depth image from the PyBullet simulation. It positions the camera based on specified positions and orientations, captures the scene, extracts the depth information, and then crops the depth image around its center. Depth images are representations where pixel values indicate distances to the objects from the camera's perspective. Such images are valuable in robotics and computer vision applications for understanding the spatial relationships and distances between objects in a scene.
        Args:
            basePos (tuple): The target or base position in the scene towards which the camera is oriented. Typically, this could be the position of an object of interest, like a table.
            cameraPos (tuple): The position coordinates (x, y, z) where the camera is placed in the simulation environment.
            cameraUp (tuple): The upward direction vector of the camera, which determines the camera's orientation. For instance, (0, 1, 0) would point the camera's upward direction along the positive y-axis.

        Returns:
            np.array: A cropped depth image centered around the midpoint of the captured image, providing depth (distance) information from the camera's perspective.
        """
        viewMatrix = p.computeViewMatrix(
            cameraPos, basePos, cameraUp, physicsClientId=self.client_id
        )
        projectionMatrix = p.computeProjectionMatrixFOV(
            fov=60,
            aspect=1.0,
            nearVal=0.01,
            farVal=100.0,
            physicsClientId=self.client_id,
        )

        img_arr = p.getCameraImage(
            640, 480, viewMatrix, projectionMatrix, shadow=1, lightDirection=[1, 1, 1]
        )
        depth_buffer = img_arr[3]  # depth buffer
        near = 0.01  # Near plane distance
        far = 100  # Far plane distance
        depth = far * near / (far - (far - near) * depth_buffer)
        # personalized colormap
        cdict = {
            "red": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
            "green": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
            "blue": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
        }
        custom_cmap = LinearSegmentedColormap("custom_cmap", cdict)
        image_point = [320, 240]

        crop_size = 200
        depth_image = self.crop_image(depth, image_point, crop_size)

        print("depth size:{} * {}".format(len(depth_image), len(depth_image[0])))
        print("image_point:{}".format(image_point))

        if enable_show:
            plt.imshow(depth_image, cmap=custom_cmap)
            plt.colorbar()
            plt.show()

        if enable_save:
            plt.imsave("depth_image.png", depth_image, cmap=custom_cmap)

        return depth_image 
