# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : Camera.py
# @Time           : 2024-08-03 15:09:34
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : Camera
"""

from datetime import datetime

import cv2
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import pybullet as p
from matplotlib.colors import LinearSegmentedColormap
from PIL import Image


class Camera:
    """Robotic Camera.

    This class handles the camera functionalities for a robotic system, including capturing RGB and depth images.
    """

    def __init__(self, cfg, base_id, arm_height, visualizer):
        """
        Initializes the Camera class with configuration, base ID, and arm height.

        Args:
            cfg (object): Configuration settings for the camera.
            base_id (int): The base ID of the robot.
            arm_height (float): The height of the robot arm.
        """
        self.base_id = base_id
        self.arm_height = arm_height
        self.visualizer = visualizer

        # projection setting
        self.fov = cfg.fov  # fov
        self.width = cfg.width  # W
        self.height = cfg.height  # H
        self.nearVal = cfg.nearVal  # nearVal
        self.farVal = cfg.farVal  # farVal

        # cameara intrinsic parameters
        self.get_focal_length()
        self.cx = self.width / 2
        self.cy = self.height / 2

        # for grasp pose
        self.head_tilt = cfg.head_tilt

        # camera rgb and depth image
        self.update()

    def get_focal_length(self):
        proj_mat = p.computeProjectionMatrixFOV(
            fov=self.fov,  # Camera's sight angle
            aspect=self.width / self.height,  # Aspect ratio of the image
            nearVal=self.nearVal,  # Camera viewing distance min
            farVal=self.farVal,  # Camera viewing distance max
        )
        self.fx = proj_mat[0] * self.width / 2
        self.fy = proj_mat[5] * self.height / 2

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
        rotation_angle = np.radians(30)
        tx_vec = self.rotate_around_y(
            np.array([r_mat[0], r_mat[3], r_mat[6]]), rotation_angle
        )  # x direction vector, the right side of the camera is facing
        tz_vec = self.rotate_around_y(
            np.array([r_mat[2], r_mat[5], r_mat[8]]), rotation_angle
        )  # z direction vector, the vertical direction of the camera

        # set the camera orientation
        target_position = camera_position + 1 * tx_vec
        self.visualizer.draw_line(camera_position, target_position)

        view_mat = p.computeViewMatrix(
            cameraEyePosition=camera_position,
            cameraTargetPosition=target_position,
            cameraUpVector=tz_vec,
        )
        up_position = camera_position + 1 * tz_vec
        self.visualizer.draw_line(camera_position, up_position)

        # A projection matrix based on the field of view (FOV) to simulate the camera's perspective
        proj_mat = p.computeProjectionMatrixFOV(
            fov=self.fov,  # Camera's sight angle
            aspect=self.width / self.height,  # Aspect ratio of the image
            nearVal=self.nearVal,  # Camera viewing distance min
            farVal=self.farVal,  # Camera viewing distance max
        )

        # width, height, rgb, image, seg
        w, h, rgb, depth, seg = p.getCameraImage(
            width=self.width,
            height=self.height,
            viewMatrix=view_mat,
            projectionMatrix=proj_mat,
            shadow=0,  # Disable Shadows
        )

        rgb = np.array(rgb, dtype=np.uint8).reshape(h, w, 4)
        depth = np.array(depth).reshape(h, w)
        
        self.image = Image.fromarray(rgb)
        self.colors = np.array(rgb[:, :, 2::-1])  # BGR to RGB

        # Convert normalized depth values ​​to actual depth values
        self.depths = (
            (
                self.farVal
                * self.nearVal
                / (self.farVal - (self.farVal - self.nearVal) * depth)
            )
            * 1000
        ).astype(np.uint16)

        return w, h, self.colors, self.depths, seg

    def get_rgb_image(self, enable_show=False, enable_save=False, filename=None):
        """
        Captures an RGB image from the camera.

        Args:
            enable_show (bool, optional): Whether to display the captured image. Defaults to False.
            enable_save (bool, optional): Whether to save the captured image. Defaults to False.

        Returns:
            np.ndarray: Captured RGB image.
        """

        _, _, rgb, _, _ = self.update()

        if enable_show:
            plt.imshow(rgb)
            plt.axis("off")
            plt.title("rgb image")
            plt.show()

        if enable_save:
            if filename is None:
                filename = "rgb_" + datetime.now().strftime("%Y%m%d_%H%M%S")
            rgb_path = f"../Examples/image/{filename}.png"
            cv2.imwrite(rgb_path, self.colors)

        return rgb

    def get_depth_image(self, enable_show=False, enable_save=False, filename=None):
        """
        Captures a depth image from the camera.

        Args:
            enable_show (bool, optional): Whether to display the captured image. Defaults to False.
            enable_save (bool, optional): Whether to save the captured image. Defaults to False.

        Returns:
            np.ndarray: Captured depth image.
        """

        _, _, _, depth, _ = self.update()

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
            plt.title("depth image")
            plt.show()

        if enable_save:
            if filename is None:
                filename = "depth_" + datetime.now().strftime("%Y%m%d_%H%M%S")
            depth_path = f"../Examples/image/{filename}.png"

            Image.fromarray(self.depths).save(depth_path)

        return depth

    # ----------------------------------------------------------------
    # functions for utils
    # ----------------------------------------------------------------

    def rotate_around_y(self, vector, angle):
        """vector rotate around y axis

        Args:
            vector (np.array): vector
            angle (): rotate angle

        Returns:
            rotated vector
        """
        rotation_matrix = np.array(
            [
                [np.cos(angle), 0, np.sin(angle)],
                [0, 1, 0],
                [-np.sin(angle), 0, np.cos(angle)],
            ]
        )
        return np.dot(rotation_matrix, vector)

    def visualize_3d_points(self):
        """visualize 3D point cloud."""
        color_image = o3d.geometry.Image(np.array(self.colors[:, :, ::-1]))
        depth_image = o3d.geometry.Image(self.depths)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_image, depth_image, convert_rgb_to_intensity=False
        )
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.Kinect2DepthCameraDefault
        )
        intrinsic.set_intrinsics(
            width=self.width,
            height=self.height,
            fx=self.fx,
            fy=self.fy,
            cx=self.cx,
            cy=self.cy,
        )
        point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, intrinsic
        )
        o3d.visualization.draw_geometries([point_cloud])

    def get_3d_points(self):
        """Convert depth image to 3D point cloud.

        Args:
            cam (CameraParameters): Camera internal parameters.

        Returns:
            np.ndarray: 3D point cloud.
        """
        xmap, ymap = np.arange(self.depths.shape[1]), np.arange(self.depths.shape[0])
        xmap, ymap = np.meshgrid(xmap, ymap)
        points_z = self.depths
        points_x = (xmap - self.cx) / self.fx * points_z
        points_y = (ymap - self.cy) / self.fy * points_z
        points = np.stack((points_x, points_y, points_z), axis=2)
        return points
