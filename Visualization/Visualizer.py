# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : Visualizer.py
# @Time           : 2024-08-03 15:09:53
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : Visualizer for pybullet 
"""

from datetime import datetime

import cv2
import numpy as np
import pybullet as p

from .utils import colors


class Visualizer:
    """
    A class for visualizing objects and scenes in PyBullet.

    Attributes:
        client (object): The PyBullet client object.
        client_id (int): The client id returned by the PyBullet client.
    """

    def __init__(self, client, visualizer_cfg):
        """
        Initializes the Visualizer class with a PyBullet client and visualizer configuration.

        Args:
            client (object): The PyBullet client object.
            visualizer_cfg (object): Configuration for the visualizer.
        """
        self.client = client
        self.client_id = client.get_client_id()
        self.set_camera_pose(visualizer_cfg.Camera)  # Init camera pose

    # ----------------------------------------------------------------
    # Scene camera
    # ----------------------------------------------------------------

    def set_camera_pose(self, camera_cfg):
        """
        Sets the debug visualizer camera pose.

        Args:
            camera_cfg (object): Configuration for the camera pose.
        """

        p.resetDebugVisualizerCamera(
            cameraDistance=camera_cfg.dist,
            cameraYaw=camera_cfg.yaw,
            cameraPitch=camera_cfg.pitch,
            cameraTargetPosition=camera_cfg.position,
            physicsClientId=self.client_id,
        )

    def capture_screen(self, filename=None):
        """
        Continuously captures the screens of PyBullet GUI and saves the images to files.

        Args:
            filename (str, optional): The filename to save the captured image. Defaults to None.
        """

        # Get GUI information
        (
            width,
            height,
            viewMatrix,
            projectionMatrix,
            _,
            _,
            _,
            _,
            _,
            _,
            _,
            _,
        ) = p.getDebugVisualizerCamera()

        # Capture the screen
        _, _, rgb, _, _ = p.getCameraImage(
            width=width,
            height=height,
            viewMatrix=viewMatrix,
            projectionMatrix=projectionMatrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
        )

        # Save the image to the file
        if filename == None:
            current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
            rgb_path = f"../Examples/image/{current_time}.png"
        else:
            rgb_path = f"../Examples/image/{filename}.png"
        rgbImg = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        cv2.imwrite(rgb_path, rgbImg)

        print("[Visualizer] \033[34mInfo\033[0m: capture_screen is done!")

    # ----------------------------------------------------------------
    # Axes
    # ----------------------------------------------------------------

    def draw_axes(self, length=1.0, lineWidth=2.0, textSize=1.0):
        """
        Draws the x, y, and z axes in the PyBullet environment with text labels.

        Args:
            length (float): Length of the axes.
            lineWidth (float): Width of the axes lines.
            textSize (float): Size of the text labels.
        """
        origin = [0, 0, 0]  # The start point of the axes

        # Drawing the X-axis (in red)
        p.addUserDebugLine(
            lineFromXYZ=origin,
            lineToXYZ=[length, 0, 0],
            lineColorRGB=[1, 0, 0],
            lineWidth=lineWidth,
            physicsClientId=self.client_id,
        )

        # Drawing the Y-axis (in green)
        p.addUserDebugLine(
            lineFromXYZ=origin,
            lineToXYZ=[0, length, 0],
            lineColorRGB=[0, 1, 0],
            lineWidth=lineWidth,
            physicsClientId=self.client_id,
        )

        # Drawing the Z-axis (in blue)
        p.addUserDebugLine(
            lineFromXYZ=origin,
            lineToXYZ=[0, 0, length],
            lineColorRGB=[0, 0, 1],
            lineWidth=lineWidth,
            physicsClientId=self.client_id,
        )

        # Adding text labels
        p.addUserDebugText(
            text="X",
            textPosition=[length + 0.1, 0, 0],
            textColorRGB=[1, 0, 0],
            textSize=textSize,
            physicsClientId=self.client_id,
        )

        p.addUserDebugText(
            text="Y",
            textPosition=[0, length + 0.1, 0],
            textColorRGB=[0, 1, 0],
            textSize=textSize,
            physicsClientId=self.client_id,
        )

        p.addUserDebugText(
            text="Z",
            textPosition=[0, 0, length + 0.1],
            textColorRGB=[0, 0, 1],
            textSize=textSize,
            physicsClientId=self.client_id,
        )

    # ----------------------------------------------------------------
    # Video
    # ----------------------------------------------------------------

    def start_record(self, fileName):
        """
        Starts recording a video of the PyBullet simulation.

        Args:
            fileName (str): The filename for the video file.
        """
        self.logId = p.startStateLogging(
            p.STATE_LOGGING_VIDEO_MP4,
            "../Examples/video/" + fileName + ".mp4",
            physicsClientId=self.client_id,
        )

        print(
            f"[Visualizer] \033[34mInfo\033[0m: The video can be found in Examples/log/{fileName}.mp4"
        )

    def end_record(self):
        """Stops recording the video."""
        p.stopStateLogging(self.logId, physicsClientId=self.client_id)

    # ----------------------------------------------------------------
    # line / aabb / pose
    # ----------------------------------------------------------------

    def draw_line(self, start_pos, target_pos, color=[1, 0, 0], width=3.0):
        """
        Draws a line on the screen from the specified start position to the target position.

        Args:
            start_pos (tuple): The starting position of the line as a tuple of (x, y, z) coordinates.
            target_pos (tuple): The ending position of the line as a tuple of (x, y, z) coordinates.
            color (list, optional): A list representing the RGB values of the line's color. Default is red [1, 0, 0].
            width (float, optional): The width of the line. Default is 3.0.
        """

        self.line_visual = p.addUserDebugLine(
            start_pos,
            target_pos,
            lineColorRGB=color,
            lineWidth=width,
            physicsClientId=self.client_id,
        )

    def remove_all_line(self):
        """Removes all user debug items (lines) from the PyBullet environment."""
        p.removeAllUserDebugItems()

    def draw_aabb(self, object):
        """
        Draws an Axis-Aligned Bounding Box (AABB) around the specified object in the simulation.

        Args:
            object (Union[int, str]): The unique identifier of the object or its name.
        """

        if isinstance(object, (int)):
            object_id = object
        elif isinstance(object, (str)):
            object_id = self.client.get_object_id(object)

        link_ids = [
            i
            for i in range(
                -1, p.getNumJoints(object_id, physicsClientId=self.client_id)
            )
        ]
        for link_id in link_ids:
            aabb = p.getAABB(object_id, link_id)
            aabb_min = aabb[0]
            aabb_max = aabb[1]
            corners = [
                [aabb_min[0], aabb_min[1], aabb_min[2]],  # 0
                [aabb_max[0], aabb_min[1], aabb_min[2]],  # 1
                [aabb_max[0], aabb_max[1], aabb_min[2]],  # 2
                [aabb_min[0], aabb_max[1], aabb_min[2]],  # 3
                [aabb_min[0], aabb_min[1], aabb_max[2]],  # 4
                [aabb_max[0], aabb_min[1], aabb_max[2]],  # 5
                [aabb_max[0], aabb_max[1], aabb_max[2]],  # 6
                [aabb_min[0], aabb_max[1], aabb_max[2]],  # 7
            ]
            lines = [
                (0, 1),
                (1, 2),
                (2, 3),
                (3, 0),  # bottom face
                (4, 5),
                (5, 6),
                (6, 7),
                (7, 4),  # top face
                (0, 4),
                (1, 5),
                (2, 6),
                (3, 7),  # vertical edges
            ]
            color = [1, 0, 0]
            for line in lines:
                p.addUserDebugLine(
                    lineFromXYZ=corners[line[0]],
                    lineToXYZ=corners[line[1]],
                    lineColorRGB=color,
                    lineWidth=2,
                    physicsClientId=self.client_id,
                )

    def draw_aabb_link(self, object, link_id=-1):
        """
        Draws an Axis-Aligned Bounding Box (AABB) around the specified link in the simulation.

        Args:
            object (Union[int, str]): The unique identifier of the object or its name.
            link_id (int, optional): The index of the link for which the AABB is to be drawn. Defaults to -1, which means the entire object.
        """
        object_id = self.client.resolve_object_id(object)

        aabb = p.getAABB(object_id, link_id, physicsClientId=self.client_id)
        aabb_min = aabb[0]
        aabb_max = aabb[1]
        corners = [
            [aabb_min[0], aabb_min[1], aabb_min[2]],  # 0
            [aabb_max[0], aabb_min[1], aabb_min[2]],  # 1
            [aabb_max[0], aabb_max[1], aabb_min[2]],  # 2
            [aabb_min[0], aabb_max[1], aabb_min[2]],  # 3
            [aabb_min[0], aabb_min[1], aabb_max[2]],  # 4
            [aabb_max[0], aabb_min[1], aabb_max[2]],  # 5
            [aabb_max[0], aabb_max[1], aabb_max[2]],  # 6
            [aabb_min[0], aabb_max[1], aabb_max[2]],  # 7
        ]
        lines = [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),  # bottom face
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),  # top face
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),  # vertical edges
        ]
        color = [1, 0, 0]
        for line in lines:
            p.addUserDebugLine(
                lineFromXYZ=corners[line[0]],
                lineToXYZ=corners[line[1]],
                lineColorRGB=color,
                lineWidth=2,
                physicsClientId=self.client_id,
            )

    def draw_object_pose(self, object, length=0.25, lineWidth=2.0, textSize=1.0):
        """
        Draws the pose of an object in the PyBullet environment.

        Args:
            object (Union[int, str]): The unique identifier of the object or its name.
            length (float, optional): The length of the pose axes. Default is 0.25.
            lineWidth (float, optional): The width of the pose axes lines. Default is 2.0.
            textSize (float, optional): The size of the text labels. Default is 1.0.
        """
        object_id = self.client.resolve_object_id(object)
        position, orientation = p.getBasePositionAndOrientation(object_id)

        orientation_matrix = np.array(p.getMatrixFromQuaternion(orientation)).reshape(
            3, 3
        )
        axes_position = np.array(position)[:, np.newaxis] + length * orientation_matrix
        text_position = (
            np.array(position)[:, np.newaxis] + (length + 0.1) * orientation_matrix
        )

        # draw pose
        p.addUserDebugLine(position, axes_position[:, 0], [1, 0, 0], lineWidth, 0)
        p.addUserDebugLine(position, axes_position[:, 1], [0, 1, 0], lineWidth, 0)
        p.addUserDebugLine(position, axes_position[:, 2], [0, 0, 1], lineWidth, 0)

        # Adding text labels
        p.addUserDebugText(
            text="X",
            textPosition=text_position[:, 0],
            textColorRGB=[1, 0, 0],
            textSize=textSize,
        )
        p.addUserDebugText(
            text="Y",
            textPosition=text_position[:, 1],
            textColorRGB=[0, 1, 0],
            textSize=textSize,
        )
        p.addUserDebugText(
            text="Z",
            textPosition=text_position[:, 2],
            textColorRGB=[0, 0, 1],
            textSize=textSize,
        )

    def draw_pose(self, pose, length=0.25, lineWidth=2.0, textSize=1.0):
        """
        Draws the pose of an object in the PyBullet environment.

        Args:
            object (Union[int, str]): The unique identifier of the object or its name.
            length (float, optional): The length of the pose axes. Default is 0.25.
            lineWidth (float, optional): The width of the pose axes lines. Default is 2.0.
            textSize (float, optional): The size of the text labels. Default is 1.0.
        """
        position = pose.get_position()
        orientation_matrix = np.array(pose.get_orientation("rotation_matrix"))
        axes_position = np.array(position)[:, np.newaxis] + length * orientation_matrix
        text_position = (
            np.array(position)[:, np.newaxis] + (length + 0.1) * orientation_matrix
        )

        # draw pose
        p.addUserDebugLine(position, axes_position[:, 0], [1, 0, 0], lineWidth, 0)
        p.addUserDebugLine(position, axes_position[:, 1], [0, 1, 0], lineWidth, 0)
        p.addUserDebugLine(position, axes_position[:, 2], [0, 0, 1], lineWidth, 0)

        # Adding text labels
        p.addUserDebugText(
            text="X",
            textPosition=text_position[:, 0],
            textColorRGB=[1, 0, 0],
            textSize=textSize,
        )
        p.addUserDebugText(
            text="Y",
            textPosition=text_position[:, 1],
            textColorRGB=[0, 1, 0],
            textSize=textSize,
        )
        p.addUserDebugText(
            text="Z",
            textPosition=text_position[:, 2],
            textColorRGB=[0, 0, 1],
            textSize=textSize,
        )

    def draw_link_pose(self, object, link_id, length=0.25, lineWidth=2.0, textSize=1.0):
        """
        Draws the pose of a specific link of an object in the PyBullet environment.

        Args:
            object (Union[int, str]): The unique identifier of the object or its name.
            link_id (int): The index of the link for which the pose is to be drawn.
            length (float, optional): The length of the pose axes. Default is 0.25.
            lineWidth (float, optional): The width of the pose axes lines. Default is 2.0.
            textSize (float, optional): The size of the text labels. Default is 1.0.
        """
        object_id = self.client.resolve_object_id(object)

        # position, orientation = p.getBasePositionAndOrientation(object_id)
        link_state = p.getLinkState(object_id, link_id, computeLinkVelocity=0)
        position, orientation = link_state[0], link_state[1]
        # position, orientation = link_state[4], link_state[5]

        orientation_matrix = np.array(p.getMatrixFromQuaternion(orientation)).reshape(
            3, 3
        )
        axes_position = np.array(position)[:, np.newaxis] + length * orientation_matrix
        text_position = (
            np.array(position)[:, np.newaxis] + (length + 0.1) * orientation_matrix
        )

        # draw pose
        p.addUserDebugLine(position, axes_position[:, 0], [1, 0, 0], lineWidth, 0)
        p.addUserDebugLine(position, axes_position[:, 1], [0, 1, 0], lineWidth, 0)
        p.addUserDebugLine(position, axes_position[:, 2], [0, 0, 1], lineWidth, 0)

        # Adding text labels
        p.addUserDebugText(
            text="X",
            textPosition=text_position[:, 0],
            textColorRGB=[1, 0, 0],
            textSize=textSize,
        )
        p.addUserDebugText(
            text="Y",
            textPosition=text_position[:, 1],
            textColorRGB=[0, 1, 0],
            textSize=textSize,
        )
        p.addUserDebugText(
            text="Z",
            textPosition=text_position[:, 2],
            textColorRGB=[0, 0, 1],
            textSize=textSize,
        )

    # ----------------------------------------------------------------
    # change color
    # ----------------------------------------------------------------

    def change_robot_color(self, base_id, arm_id, light_color=True):
        """
        Sets the color of the robot.

        Args:
            base_id (int): The unique identifier of the robot base.
            arm_id (int): The unique identifier of the robot arm.
            light_color (bool, optional): Flag to set the robot to light colors. Default is True.
        """

        # set the light white color to base
        base_num_joints = p.getNumJoints(base_id, physicsClientId=self.client_id)
        for i in range(base_num_joints):
            p.changeVisualShape(
                objectUniqueId=base_id,
                linkIndex=i,
                rgbaColor=colors["light_white"],
                physicsClientId=self.client_id,
            )
            if self.client.blender:
                self.client.mtl_recorder[f"{base_id}{i}"] = colors["light_white"]

        # set the blue and white color to arm
        arm_num_joints = p.getNumJoints(arm_id, physicsClientId=self.client_id)
        for i in range(arm_num_joints):
            if i % 3 == 0:
                if light_color:
                    p.changeVisualShape(
                        objectUniqueId=arm_id,
                        linkIndex=i,
                        rgbaColor=colors["light_blue"],
                        physicsClientId=self.client_id,
                    )
                    if self.client.blender:
                        self.client.mtl_recorder[f"{arm_id}{i}"] = colors["light_blue"]
                else:
                    p.changeVisualShape(
                        objectUniqueId=arm_id,
                        linkIndex=i,
                        rgbaColor=colors["blue"],
                        physicsClientId=self.client_id,
                    )
                    if self.client.blender:
                        self.client.mtl_recorder[f"{arm_id}{i}"] = colors["blue"]
            else:
                p.changeVisualShape(
                    objectUniqueId=arm_id,
                    linkIndex=i,
                    rgbaColor=colors["light_white"],
                    physicsClientId=self.client_id,
                )
                if self.client.blender:
                    self.client.mtl_recorder[f"{arm_id}{i}"] = colors["light_white"]

    def set_object_color(self, object_id, color):
        """
        Sets the color of an object.

        Args:
            object_id (int): The unique identifier of the object.
            color (str): The color to set for the object.
        """
        object_num_joints = p.getNumJoints(object_id, physicsClientId=self.client_id)
        for i in range(object_num_joints):
            p.changeVisualShape(
                objectUniqueId=object_id,
                linkIndex=i,
                rgbaColor=colors[color],
                physicsClientId=self.client_id,
            )
        # p.changeVisualShape(
        #     objectUniqueId=object_id,
        #     linkIndex=-1,
        #     rgbaColor=colors[color],
        #     physicsClientId=self.client_id
        # )
        if self.client.blender:
            self.client.mtl_recorder[f"{object_id}"] = colors[color]

    def set_link_color(self, object_id, link_id, color):
        """
        Sets the color of a specific link of an object.

        Args:
            object_id (int): The unique identifier of the object.
            link_id (int): The index of the link to change the color.
            color (str): The color to set for the link.
        """
        p.changeVisualShape(
            objectUniqueId=object_id,
            linkIndex=link_id,
            rgbaColor=colors[color],
            physicsClientId=self.client_id,
        )
        if self.client.blender:
            self.client.mtl_recorder[f"{object_id}{link_id}"] = colors[color]

    def set_links_color(self, object_id, link_ids, colors):
        """
        Sets the colors of multiple links of an object.

        Args:
            object_id (int): The unique identifier of the object.
            link_ids (list[int]): A list of link indexes to change the colors.
            colors (list[str]): A list of colors to set for the links.
        """
        for i, color in zip(link_ids, colors):
            p.changeVisualShape(
                objectUniqueId=object_id,
                linkIndex=i,
                rgbaColor=colors[color],
                physicsClientId=self.client_id,
            )
            if self.client.blender:
                self.client.mtl_recorder[f"{object_id}{i}"] = colors[color]
