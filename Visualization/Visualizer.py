"""
@Description :   A few functions for visualization
@Author      :   Yan Ding 
@Time        :   2023/08/30 22:48:04
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
from PIL import Image
from datetime import datetime

"""
Visualization class
"""

# color list
colors = {
    "light_white": [1, 1, 1, 1],
    "wood_light": [0.87, 0.72, 0.53, 1],
    "wood_heavy": [0.4, 0.26, 0.13, 1],
    "stainless_steel": [0.59, 0.59, 0.59, 1],
    "wood_dark": [0.4, 0.26, 0.13, 1],
    "cream": [1, 0.99, 0.82, 1],
    "light_grey": [0.9, 0.9, 0.9, 1],
    "grey": [0.56, 0.56, 0.56, 1],
    "dark_grey": [0.13, 0.13, 0.13, 1],
    "blue": [0.53, 0.81, 0.92, 1.0],
    "light_blue": [0.9, 0.9, 1, 1],
    "light_white": [0.98, 0.98, 0.98, 1.0],
    "white": [1, 1, 1, 1.0],
    "black": [0, 0, 0, 1],
}


class Visualizer:
    def __init__(self, client, visualizer_cfg):
        self.client = client
        self.client_id = client.get_client_id()
        
        # Init camera pose
        self.set_camera_pose(visualizer_cfg.Camera)
        self.line_visual = None

    # ----------------------------------------------------------------
    # Camera / Image
    # ----------------------------------------------------------------
    
    def set_camera_pose(self, camera_cfg):
        """
        Set the debug visualizer camera pose 

        Args:
            dist (float): The distance of the camera from the target point.
            position (list): A list of three floats representing the target position in 3D space.
            yaw (float, optional): The yaw component of the camera orientation. Defaults to 0.
            pitch (float, optional): The pitch component of the camera orientation. Defaults to -89.9.
        """
        
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_cfg.dist,
            cameraYaw=camera_cfg.yaw,
            cameraPitch=camera_cfg.pitch,
            cameraTargetPosition=camera_cfg.position,
            physicsClientId=self.client_id,
        )
    
    def capture_screen(self, filename=None, enable_Debug=False):
        """
        Continuously capture the screens of pybullet GUI and save the images to files.
        The file names will be based on the camera target position, distance, and yaw.

        Parameters:
        width (int): The width of the captured image.
        height (int): The height of the captured image.
        """
        alpha = 10
        temp_counter = 0
        max_counter = 1  # num of images
        
        if enable_Debug:
            while temp_counter < max_counter:
                # Get GUI information
                (
                    width,
                    height,
                    viewMatrix,
                    projectionMatrix,
                    cameraUp,
                    camForward,
                    horizonal,
                    vertical,
                    yaw,
                    pitch,
                    dist,
                    target,
                ) = p.getDebugVisualizerCamera()

                # Capture the screen
                _, _, rgba, _, _ = p.getCameraImage(width * alpha, height * alpha)
                img = np.array(rgba).reshape(height * alpha, width * alpha, 4)

                # Save the image to the file
                if filename == None:
                    # current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
                    # path_filename = f"./image/{current_time}.png"
                    path_filename = f"./image/input/target:{target}_dist:{dist}_pitch:{pitch}_yaw:{yaw}.png"
                else:
                    path_filename = f"./image/input/{filename}.png"
                Image.fromarray(img).save(path_filename)
                temp_counter += 1
                print("-" * 20 + "image is done!" + "-" * 20)
        else:
            # Get GUI information
            (
                width,
                height,
                viewMatrix,
                projectionMatrix,
                cameraUp,
                camForward,
                horizonal,
                vertical,
                yaw,
                pitch,
                dist,
                target,
            ) = p.getDebugVisualizerCamera()

            # Capture the screen
            _, _, rgba, _, _ = p.getCameraImage(width * alpha, height * alpha)
            img = np.array(rgba).reshape(height * alpha, width * alpha, 4)

            print("width:{} height:{}".format(width, height))

            # Save the image to the file
            if filename == None:
                # current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
                # path_filename = f"./image/{current_time}.png"
                path_filename = f"./image/input/target:{target}_dist:{dist}_pitch:{pitch}_yaw:{yaw}.png"
            else:
                path_filename = f"./image/input/{filename}.png"
            Image.fromarray(img).save(path_filename)
    
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
    
    def crop_image(self, image, center, size):
        """
        Crop a given image around a specified center point and returns the cropped portion. The resulting cropped image will be a square with the provided size. If the cropping dimensions exceed the original image boundaries, the function ensures it stays within the original image's dimensions to prevent out-of-bounds access.

        Args:
            image (np.array): The original image to be cropped. It is assumed to be a two-dimensional array, representing pixel values.
            center (tuple): A tuple (x, y) specifying the center point around which the image will be cropped.
            size (int): The side length of the resulting square cropped image.

        Returns:
            np.array: A cropped portion of the original image centered around the specified center point and with the given size.
        """
        image_height, image_width = image.shape
        top = max(0, int(center[1] - size / 2))
        bottom = min(image_height, int(center[1] + size / 2))
        left = max(0, int(center[0] - size / 2))
        right = min(image_width, int(center[0] + size / 2))

        return image[top:bottom, left:right]
            
            
    # ----------------------------------------------------------------
    # Axes
    # ----------------------------------------------------------------
    def draw_axes(self, length=2.0, lineWidth=2.0, textSize=3.0):
        """
        Draws the x and y axes in the PyBullet environment with text labels.
        
        Parameters:
        - length (float): Length of the axes.
        - lineWidth (float): Width of the axes lines.
        - textSize (float): Size of the text labels.
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
    
    
    # ----------------------------------------------------------------
    # Video
    # ----------------------------------------------------------------

    def start_record(self, fileName):
        """
        Enable and disable recording
        """
        
        logId = p.startStateLogging(
            p.STATE_LOGGING_VIDEO_MP4,
            "./log/" + fileName + ".mp4",
            physicsClientId=self.client_id,
        )
        
        print(
            "-" * 20
            + "\n"
            + "The video can be found in "
            + "./log/"
            + fileName
            + ".mp4"
        )
        
        return logId

    def end_record(self, logId):
        p.stopStateLogging(logId, physicsClientId=self.client_id)
        
        
    # ----------------------------------------------------------------
    # aabb
    # ----------------------------------------------------------------

    def draw_aabb(self, object_id):
        """
        Draw an Axis-Aligned Bounding Box (AABB) around the specified table object in the simulation. The AABB is a box that covers the entire object based on its maximum and minimum coordinates along each axis. It can be useful for various purposes, such as collision detection, spatial partitioning, and bounding volume hierarchies.

        Args:
            table_id: The unique identifier of the table object in the simulation for which the AABB is to be drawn.
        """
        link_ids = [
            i
            for i in range(
                -1, p.getNumJoints(object_id, physicsClientId=self.client_id)
            )
        ]
        print(f"test: {link_ids}")
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
        Draw an Axis-Aligned Bounding Box (AABB) around the specified object or link in the simulation. The AABB is a box that covers the entire object based on its maximum and minimum coordinates along each axis. It can be useful for various purposes, such as collision detection, spatial partitioning, and bounding volume hierarchies.

        Args:
            object_id: The unique identifier of the object in the simulation for which the AABB is to be drawn.
            link_id: The index of the link for which the AABB is to be drawn. Default is -1, which means the entire object.
        """
        
        if isinstance(object, (int)):   # input type is object id
            object_id = object
        elif isinstance(object, (str)): # input type is object name
            object_id = getattr(self.client, object)
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


    # Robot color
    def change_robot_color(self, base_id, arm_id, light_color=True):
        """
        Set the color of robot
        """

        # set the color of base
        base_num_joints = p.getNumJoints(base_id, physicsClientId=self.client_id)
        for i in range(base_num_joints):
            p.changeVisualShape(
                objectUniqueId=base_id,
                linkIndex=i,
                rgbaColor=colors["white"],
                physicsClientId=self.client_id,
            )
        
        # set the color of arm
        arm_num_joints = p.getNumJoints(arm_id, physicsClientId=self.client_id)
        for i in range(arm_num_joints):
            if i % 3 == 0:
                if light_color:
                    p.changeVisualShape(
                        objectUniqueId=arm_id,
                        linkIndex=i,
                        rgbaColor=colors["light_blue"],
                        physicsClientId=self.client_id
                    )
                else:
                    p.changeVisualShape(
                        objectUniqueId=arm_id,
                        linkIndex=i,
                        rgbaColor=colors["blue"],
                        physicsClientId=self.client_id
                    )
            else:
                p.changeVisualShape(
                    objectUniqueId=arm_id, 
                    linkIndex=i, 
                    rgbaColor=colors["light_white"], 
                    physicsClientId=self.client_id
                )

    def set_elementA_visual_color(self, elementA_id):
        """
        Set the color of element A (oven, container) in the kitchen.
        """
        for i in [1]:  # center counter
            p.changeVisualShape(
                objectUniqueId=elementA_id,
                linkIndex=i,
                rgbaColor=colors["light_white"],
                physicsClientId=self.client_id,
            )
        for i in [2]:  # oven base
            p.changeVisualShape(
                objectUniqueId=elementA_id,
                linkIndex=i,
                rgbaColor=colors["light_white"],
                physicsClientId=self.client_id,
            )
        for i in [3, 4, 5, 6]:  # four ovens
            p.changeVisualShape(
                objectUniqueId=elementA_id,
                linkIndex=i,
                rgbaColor=colors["stainless_steel"],
                physicsClientId=self.client_id,
            )
        for i in [7]:  # rectangle where buttons are on
            p.changeVisualShape(
                objectUniqueId=elementA_id,
                linkIndex=i,
                rgbaColor=colors["wood_dark"],
                physicsClientId=self.client_id,
            )
        for i in [8, 9, 10, 11]:  # four buttons
            p.changeVisualShape(
                objectUniqueId=elementA_id,
                linkIndex=i,
                rgbaColor=colors["black"],
                physicsClientId=self.client_id,
            )
        for i in [30, 33, 34, 42, 43]:  # left and right counter
            p.changeVisualShape(
                objectUniqueId=elementA_id,
                linkIndex=i,
                rgbaColor=colors["light_white"],
                physicsClientId=self.client_id,
            )
    
    def set_elementB_visual_color(self, elementB_id):
        """
        Set the color of element B (counter) in the kitchen.
        """
        # link id:
        for i in [-1]:  # wood_dark
            p.changeVisualShape(
                objectUniqueId=elementB_id,
                linkIndex=i,
                rgbaColor=colors["light_white"],
                physicsClientId=self.client_id,
            )

    def set_elementC_visual_color(self, elementC_id):
        """
        Set the color of element C (dishwasher) in the kitchen.
        """
        # link id:
        for i in [-1, 0, 1, 2, 3, 4, 5]:
            p.changeVisualShape(
                objectUniqueId=elementC_id,
                linkIndex=i,
                rgbaColor=colors["light_white"],
                physicsClientId=self.client_id,
            )

    def set_elementD_visual_color(self, elementD_id):
        """
        Set the color of element D (microwave) in the kitchen.
        """
        # link id:
        for i in [0]:  # back
            p.changeVisualShape(
                objectUniqueId=elementD_id,
                linkIndex=i,
                rgbaColor=colors["light_white"],
                physicsClientId=self.client_id,
            )

        # link id:
        for i in [1]:  # door
            p.changeVisualShape(
                objectUniqueId=elementD_id,
                linkIndex=i,
                rgbaColor=colors["stainless_steel"],
                physicsClientId=self.client_id,
            )

    def set_elementE_visual_color(self, elementE_id):
        """
        Set the color of element E (fridge) in the kitchen.
        """
        # link id:
        for i in [-1, 0, 1, 2, 3, 4, 5, 6]:  # wood_light
            p.changeVisualShape(
                objectUniqueId=elementE_id,
                linkIndex=i,
                rgbaColor=colors["light_white"],
                physicsClientId=self.client_id,
            )

    def set_elementF_visual_color(self, elementF_id):
        """
        Set the color of element F (table) in the kitchen.
        """
        # link id:
        for i in [-1, 0, 1, 2, 3]:
            p.changeVisualShape(
                objectUniqueId=elementF_id,
                linkIndex=i,
                rgbaColor=colors["light_white"],
                physicsClientId=self.client_id,
            )

    def set_elementG_visual_color(self, elementG_id):
        """
        Set the color of element G (Chair) in the kitchen.
        """
        # link id:
        for i in [-1]:  # light_white
            p.changeVisualShape(
                objectUniqueId=elementG_id,
                linkIndex=i,
                rgbaColor=colors["white"],
                physicsClientId=self.client_id,
            )

    def visualize_path(self, path):
        # Reverse the path so that it goes from start to goal
        path = path[::-1]  # This line reverses the path
        cartesian_path = [self.joints_to_cartesian(point) for point in path]
        # print("cartesian_path:{}".format(cartesian_path))
        for i in range(len(cartesian_path) - 1):
            p.addUserDebugLine(
                cartesian_path[i],
                cartesian_path[i + 1],
                lineColorRGB=[1, 0, 0],
                lifeTime=10,
                lineWidth=3,
                physicsClientId=self.client_id,
            )

    def draw_line(self, start_pos, target_pos, color=[1, 0, 0], width=3.0):
        """
        Draw a line on the screen from the specified start position to the target position.

        Args:
            start_pos: The starting position of the line as a tuple of (x, y, z) coordinates.
            target_pos: The ending position of the line as a tuple of (x, y, z) coordinates.
            color: A list representing the RGB values of the line's color. Default is red [1, 0, 0].
            width: The width of the line. Default is 3.0.
        """
        if self.line_visual is not None:
            p.removeUserDebugItem(self.line_visual, physicsClientId=self.client_id)

        self.line_visual = p.addUserDebugLine(
            start_pos,
            target_pos,
            lineColorRGB=color,
            lineWidth=width,
            physicsClientId=self.client_id,
        )
