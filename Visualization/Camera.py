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
from RoboticsToolBox.Pose import Pose


class Camera:
    """Robotic Camera.

    This class handles the camera functionalities for a robotic system, including capturing RGB and depth images.
    """

    def __init__(self, cfg, base_id, arm_place_height):
        """
        Initializes the Camera class with configuration, base ID, and arm height.

        Args:
            cfg (object): Configuration settings for the camera.
            base_id (int): The base ID of the robot.
            arm_height (float): The height of the robot arm.
        """
        self.base_id = base_id
        self.arm_place_height = arm_place_height

        # projection setting
        self.fov = cfg.fov  # fov
        self.width = cfg.width  # W
        self.height = cfg.height  # H
        self.nearVal = cfg.nearVal  # nearVal
        self.farVal = cfg.farVal  # farVal

        # for grasp pose
        self.head_tilt = cfg.head_tilt

        # camera rgb and depth image
        self.sim_update()
        
        # cameara intrinsic parameters
        self.sim_get_focal_length()
        self.cx = self.width / 2
        self.cy = self.height / 2

    def sim_get_focal_length(self):
        """
        get camera focal length
        """
        self.fx = self.proj_mat[0] * self.width / 2
        self.fy = self.proj_mat[5] * self.height / 2

    def sim_get_camera_pose(self):
        """
        get camera pose
        """
        view_mat = np.array(self.trans_camera_to_world).reshape([4, 4], order="F")
        pose = Pose(view_mat[:3, -1], view_mat[:3, :3])
        return pose
    
    def sim_update(self):
        """
        Updates the camera view and projection matrices, captures images, and returns the captured data.

        Returns:
            tuple: Width, height, RGB image, depth image, and segmentation mask.
        """

        # get base pose
        position, orientation = p.getBasePositionAndOrientation(self.base_id)
        camera_position = np.array([position[0] + 0.5, position[1], self.arm_place_height + 0.3])

        # The three direction vectors of the camera in the world coordinate system
        # r_mat = p.getMatrixFromQuaternion(orientation)
        # rotation_angle = np.radians(90)
        # tx_vec = self.sim_rotate_around_y(
        #     np.array([r_mat[0], r_mat[3], r_mat[6]]), rotation_angle
        # )  # x direction vector, the right side of the camera is facing
        # tz_vec = self.sim_rotate_around_y(
        #     np.array([r_mat[2], r_mat[5], r_mat[8]]), rotation_angle
        # )  # z direction vector, the vertical direction of the camera
        

        # set the camera orientation
        target_position = camera_position +  [0, 0, -1]

        self.view_mat = p.computeViewMatrix(
            cameraEyePosition=camera_position,
            cameraTargetPosition=target_position,
            cameraUpVector=[1, 0, 0]
        )
        
        # update trans camera to world mat
        self.trans_camera_to_world = np.linalg.inv(np.array(self.view_mat).reshape([4, 4], order="F"))

        # A projection matrix based on the field of view (FOV) to simulate the camera's perspective
        self.proj_mat = p.computeProjectionMatrixFOV(
            fov=self.fov,  # Camera's sight angle
            aspect=self.width / self.height,  # Aspect ratio of the image
            nearVal=self.nearVal,  # Camera viewing distance min
            farVal=self.farVal,  # Camera viewing distance max
        )

        # width, height, rgb, image, seg
        w, h, rgb, depth, _ = p.getCameraImage(
            width=self.width,
            height=self.height,
            viewMatrix=self.view_mat,
            projectionMatrix=self.proj_mat
        )

        # make sure the array has the correct shape
        rgb = np.array(rgb, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
        depth = np.array(depth).reshape(h, w)

        self.image = Image.fromarray(rgb)
        self.colors = np.array(rgb)  # BGR to RGB

        # pybullet return normalized depth values, convert ​​to actual depth values, unit：m
        self.depths = self.farVal * self.nearVal / (self.farVal - (self.farVal - self.nearVal) * depth)


    def sim_get_rgb_image(self, enable_show=False, enable_save=False, filename=None):
        """
        Captures an RGB image from the camera.

        Args:
            enable_show (bool, optional): Whether to display the captured image. Defaults to False.
            enable_save (bool, optional): Whether to save the captured image. Defaults to False.

        Returns:
            np.ndarray: Captured RGB image.
        """

        if enable_show:
            plt.imshow(self.colors)
            plt.axis("off")
            plt.title("rgb image")
            plt.show()

        if enable_save:
            if filename is None:
                filename = "rgb_" + datetime.now().strftime("%Y%m%d_%H%M%S")
            rgb_path = f"../Examples/image/{filename}.png"
            new_image = cv2.cvtColor(self.colors, cv2.COLOR_BGR2RGB)
            cv2.imwrite(rgb_path, new_image)

        return self.colors

    def sim_get_depth_image(self, enable_show=False, enable_save=False, filename=None):
        """
        Captures a depth image from the camera.

        Args:
            enable_show (bool, optional): Whether to display the captured image. Defaults to False.
            enable_save (bool, optional): Whether to save the captured image. Defaults to False.

        Returns:
            np.ndarray: Captured depth image.
        """

        if enable_show:
            # Linear color mapping from black (the color when the depth value is 0) to white (the color when the depth value is 1)
            cdict = {
                "red": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
                "green": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
                "blue": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
            }
            custom_cmap = LinearSegmentedColormap("custom_cmap", cdict)
            plt.imshow(self.depths, cmap=custom_cmap)
            plt.colorbar()
            plt.title("depth image")
            plt.show()

        if enable_save:
            if filename is None:
                filename = "depth_" + datetime.now().strftime("%Y%m%d_%H%M%S")
            depth_path = f"../Examples/image/{filename}.png"
            depths = (self.depths * 1000).astype(np.uint16)
            Image.fromarray(depths).save(depth_path)
        
        return self.depths

    # ----------------------------------------------------------------
    # functions for utils
    # ----------------------------------------------------------------

    def sim_rotate_around_y(self, vector, angle):
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

    def sim_visualize_3d_points(self):
        """
        visualize 3D point cloud.
        """
        color_image = o3d.geometry.Image(np.array(self.colors))
        depth_image = o3d.geometry.Image((self.depths * 1000).astype(np.uint16))
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
        trans_mat = np.array(
                [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
            )
        point_cloud.transform(trans_mat)
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="visualize camera 3d points")
        vis.add_geometry(point_cloud)
        vis.add_geometry(coordinate_frame)
        vis.run()
        vis.destroy_window()

    def sim_get_3d_points(self, filter_dist=[0, 1]):
        """
        Convert depth image to 3D point cloud.

        Returns:
            np.ndarray: 3D point cloud.
        """
        xmap, ymap = np.arange(self.depths.shape[1]), np.arange(self.depths.shape[0])
        xmap, ymap = np.meshgrid(xmap, ymap)
        points_z = self.depths
        points_x = (xmap - self.cx) / self.fx * points_z
        points_y = (ymap - self.cy) / self.fy * points_z
        mask = (points_z > filter_dist[0]) & (points_z < filter_dist[1])
        points = np.stack([points_x, -points_y, points_z], axis=-1)
        points = points[mask].astype(np.float32)
        colors = (self.colors / 255.0)[mask].astype(np.float32)
        return points, colors
    
    def sim_trans_to_world(self, pose):
        """
        Convert grasp pose from camera to world coord system 

        Args:
            pose (Pose): pose in camera coord system
        
        Returns:
            Pose: pose in world system
        """
        pose_mat = np.eye(4)
        # pose_mat[:3, -1] = pose[0]
        # pose_mat[:3, :3] = pose[1]
        pose_mat[:3, -1] = pose.get_position()
        pose_mat[:3, :3] = pose.get_orientation("rotation_matrix")
        trans_mat = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
        )
        world_pose = self.trans_camera_to_world @ (trans_mat @ pose_mat)
        world_pose[2, 3] -= 0.03
        world_pose = Pose(world_pose[:3, -1], world_pose[:3, :3])
        return world_pose
